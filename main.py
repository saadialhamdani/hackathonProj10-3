
"""STM32F401RE robot path-drawing app.

Install: python -m pip install pyserial
Run:     python main.py

Firmware protocol: F,<milliseconds>, L,<milliseconds>, R,<milliseconds>, S
Each command must receive DONE before the next command is sent.
"""

import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    raise SystemExit("PySerial is missing. Run: python -m pip install pyserial")

GRID_METERS = 0.2
GRID_LIMIT = 50                 # -50..50 steps = -10..10 metres
MAX_COMMAND_MS = 5000          # Matches STM32 firmware
BAUD = 9600
DIRECTIONS = {(1, 0): 0, (0, 1): 1, (-1, 0): 2, (0, -1): 3}


def build_commands(points, ms_per_meter, turn_ms):
    """Convert orthogonal grid-step points into STM32 movement commands."""
    if ms_per_meter <= 0 or turn_ms <= 0 or turn_ms > MAX_COMMAND_MS:
        raise ValueError("Use positive calibration values; turn time must be <= 5000 ms.")

    commands = []
    heading = 0  # Robot starts facing east (right on the screen).

    for (x1, y1), (x2, y2) in zip(points, points[1:]):
        dx, dy = x2 - x1, y2 - y1
        if not (dx == 0 or dy == 0) or (dx == 0 and dy == 0):
            raise ValueError("Path must consist of nonzero horizontal/vertical segments.")

        step = (0 if dx == 0 else (1 if dx > 0 else -1),
                0 if dy == 0 else (1 if dy > 0 else -1))
        next_heading = DIRECTIONS[step]
        rotation = (next_heading - heading) % 4
        if rotation == 1:
            commands.append(f"L,{turn_ms}")
        elif rotation == 2:
            commands.extend((f"L,{turn_ms}", f"L,{turn_ms}"))
        elif rotation == 3:
            commands.append(f"R,{turn_ms}")
        heading = next_heading

        distance_metres = (abs(dx) + abs(dy)) * GRID_METERS
        remaining_ms = max(1, round(distance_metres * ms_per_meter))
        while remaining_ms:
            portion = min(remaining_ms, MAX_COMMAND_MS)
            commands.append(f"F,{portion}")
            remaining_ms -= portion

    return commands


class RobotApp:
    def __init__(self, root):
        self.root = root
        root.title("STM32 Robot - Path Drawing")
        root.geometry("1200x800")
        try:
            root.state("zoomed")           # Maximize on Windows.
        except tk.TclError:
            pass
        root.bind("<F11>", self.toggle_fullscreen)
        root.bind("<Escape>", lambda _event: root.attributes("-fullscreen", False))
        root.protocol("WM_DELETE_WINDOW", self.close)

        self.points = [(0, 0)]           # Coordinates are integer 20 cm steps.
        self.history = []
        self.scale = 35.0                # Pixels per metre.
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.drag_at = None
        self.ser = None
        self.serial_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.events = queue.Queue()
        self.running = False

        self.port_var = tk.StringVar(value="COM4")
        self.meter_var = tk.StringVar(value="5000")
        self.turn_var = tk.StringVar(value="500")
        self.status_var = tk.StringVar(value="Disconnected")
        self.position_var = tk.StringVar(value="End point: (0.0, 0.0) m")

        panel = ttk.Frame(root, padding=12)
        panel.pack(side="left", fill="y")
        ttk.Label(panel, text="ROBOT PATH CONTROL", font=("Arial", 14, "bold")).pack(anchor="w", pady=(0, 16))

        ttk.Label(panel, text="HC-05 outgoing COM port").pack(anchor="w")
        self.port_box = ttk.Combobox(panel, textvariable=self.port_var, width=19)
        self.port_box.pack(fill="x", pady=(3, 5))
        ttk.Button(panel, text="Refresh ports", command=self.refresh_ports).pack(fill="x")
        self.connect_btn = ttk.Button(panel, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(fill="x", pady=(6, 4))
        self.test_btn = ttk.Button(panel, text="Test (send S)", command=self.test_connection)
        self.test_btn.pack(fill="x", pady=(0, 12))

        ttk.Label(panel, text="Forward milliseconds / metre").pack(anchor="w")
        ttk.Entry(panel, textvariable=self.meter_var, width=20).pack(fill="x", pady=(2, 9))
        ttk.Label(panel, text="90-degree turn milliseconds").pack(anchor="w")
        ttk.Entry(panel, textvariable=self.turn_var, width=20).pack(fill="x", pady=(2, 12))

        self.run_btn = ttk.Button(panel, text="RUN PATH", command=self.run_path)
        self.run_btn.pack(fill="x", pady=3)
        self.stop_btn = ttk.Button(panel, text="STOP MOTORS", command=self.stop_robot)
        self.stop_btn.pack(fill="x", pady=3)
        self.undo_btn = ttk.Button(panel, text="Undo last click", command=self.undo)
        self.undo_btn.pack(fill="x", pady=(14, 3))
        self.clear_btn = ttk.Button(panel, text="Clear path", command=self.clear)
        self.clear_btn.pack(fill="x", pady=3)

        ttk.Separator(panel).pack(fill="x", pady=12)
        ttk.Label(panel, textvariable=self.position_var, wraplength=230).pack(anchor="w")
        ttk.Label(panel, text="Commands to send:").pack(anchor="w", pady=(12, 3))
        self.preview = tk.Text(panel, width=25, height=13, state="disabled", font=("Consolas", 10))
        self.preview.pack(fill="both", expand=True)
        ttk.Label(panel, textvariable=self.status_var, wraplength=230,
                  foreground="#146c43").pack(anchor="w", pady=(12, 4))
        ttk.Label(panel,
                  text="Start robot at (0,0), facing right.\n"
                       "Left-click: draw path\nRight-drag: pan\nMouse wheel: zoom\nF11: fullscreen / Esc: exit",
                  wraplength=230, justify="left").pack(anchor="w", pady=(8, 0))

        self.canvas = tk.Canvas(root, background="white", highlightthickness=0)
        self.canvas.pack(side="right", fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda _event: self.draw())
        self.canvas.bind("<Button-1>", self.add_point)
        self.canvas.bind("<ButtonPress-3>", self.pan_start)
        self.canvas.bind("<B3-Motion>", self.pan_move)
        self.canvas.bind("<MouseWheel>", self.zoom)         # Windows and macOS
        self.canvas.bind("<Button-4>", lambda e: self.zoom(e, 1))  # Linux
        self.canvas.bind("<Button-5>", lambda e: self.zoom(e, -1))

        self.refresh_ports()
        self.update_preview()
        self.update_buttons()
        root.after(50, self.process_events)

    def toggle_fullscreen(self, _event=None):
        self.root.attributes("-fullscreen", not self.root.attributes("-fullscreen"))

    def refresh_ports(self):
        ports = sorted(p.device for p in list_ports.comports())
        self.port_box["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def toggle_connection(self):
        if self.running:
            return
        if self.ser is not None:
            with self.serial_lock:
                try:
                    self.ser.write(b"S\n")
                except serial.SerialException:
                    pass
                self.ser.close()
                self.ser = None
            self.status_var.set("Disconnected")
        else:
            try:
                self.ser = serial.Serial(self.port_var.get().strip(), BAUD,
                                         timeout=0.1, write_timeout=1)
                self.ser.reset_input_buffer()
                self.status_var.set(f"Connected: {self.ser.port} @ {BAUD}")
            except (serial.SerialException, ValueError) as exc:
                self.ser = None
                messagebox.showerror("Bluetooth connection", str(exc))
        self.update_buttons()

    def update_buttons(self):
        connected = self.ser is not None
        self.connect_btn.configure(text="Disconnect" if connected else "Connect",
                                   state="disabled" if self.running else "normal")
        self.test_btn.configure(state="normal" if connected and not self.running else "disabled")
        self.run_btn.configure(state="normal" if connected and not self.running
                               and len(self.points) > 1 else "disabled")
        self.stop_btn.configure(state="normal" if connected else "disabled")
        self.undo_btn.configure(state="normal" if self.history and not self.running else "disabled")
        self.clear_btn.configure(state="normal" if not self.running else "disabled")
        self.port_box.configure(state="disabled" if connected else "normal")

    def world_to_screen(self, step_x, step_y):
        w, h = self.canvas.winfo_width(), self.canvas.winfo_height()
        return (w / 2 + step_x * GRID_METERS * self.scale + self.pan_x,
                h / 2 - step_y * GRID_METERS * self.scale + self.pan_y)

    def screen_to_step(self, px, py):
        w, h = self.canvas.winfo_width(), self.canvas.winfo_height()
        x = round((px - w / 2 - self.pan_x) / (GRID_METERS * self.scale))
        y = round((h / 2 + self.pan_y - py) / (GRID_METERS * self.scale))
        return max(-GRID_LIMIT, min(GRID_LIMIT, x)), max(-GRID_LIMIT, min(GRID_LIMIT, y))

    def draw(self):
        c = self.canvas
        c.delete("all")
        for n in range(-GRID_LIMIT, GRID_LIMIT + 1):
            x1, y1 = self.world_to_screen(n, -GRID_LIMIT)
            x2, y2 = self.world_to_screen(n, GRID_LIMIT)
            if n == 0:
                color, width = "#64748b", 2
            elif n % 5 == 0:
                color, width = "#cad5e2", 1
            else:
                color, width = "#edf2f7", 1
            c.create_line(x1, y1, x2, y2, fill=color, width=width)
            x1, y1 = self.world_to_screen(-GRID_LIMIT, n)
            x2, y2 = self.world_to_screen(GRID_LIMIT, n)
            c.create_line(x1, y1, x2, y2, fill=color, width=width)

            if n % 5 == 0 and n != 0:
                label_x, axis_y = self.world_to_screen(n, 0)
                axis_x, label_y = self.world_to_screen(0, n)
                c.create_text(label_x, axis_y + 11, text=str(n // 5),
                              fill="#475569", font=("Arial", 8))
                c.create_text(axis_x - 13, label_y, text=str(n // 5),
                              fill="#475569", font=("Arial", 8))

        x1, y1 = self.world_to_screen(-GRID_LIMIT, GRID_LIMIT)
        x2, y2 = self.world_to_screen(GRID_LIMIT, -GRID_LIMIT)
        c.create_rectangle(x1, y1, x2, y2, outline="#475569", width=2)
        origin_x, origin_y = self.world_to_screen(0, 0)
        c.create_text(origin_x + 13, origin_y + 12, text="0", fill="#334155")

        for a, b in zip(self.points, self.points[1:]):
            ax, ay = self.world_to_screen(*a)
            bx, by = self.world_to_screen(*b)
            c.create_line(ax, ay, bx, by, fill="#ea580c", width=4)

        for point in self.points[1:-1]:
            x, y = self.world_to_screen(*point)
            c.create_oval(x - 3, y - 3, x + 3, y + 3,
                          fill="#ea580c", outline="white")

        x, y = self.world_to_screen(*self.points[0])
        c.create_oval(x - 6, y - 6, x + 6, y + 6,
                      fill="#16a34a", outline="white", width=2)
        if len(self.points) > 1:
            x, y = self.world_to_screen(*self.points[-1])
            c.create_oval(x - 6, y - 6, x + 6, y + 6,
                          fill="#dc2626", outline="white", width=2)

    def add_point(self, event):
        if self.running:
            return
        target_x, target_y = self.screen_to_step(event.x, event.y)
        old_length = len(self.points)
        last_x, last_y = self.points[-1]
        # Always travel horizontally first, then vertically.
        if target_x != last_x:
            self.points.append((target_x, last_y))
        if target_y != last_y:
            self.points.append((target_x, target_y))
        if len(self.points) > old_length:
            self.history.append(old_length)
            self.update_preview()
            self.draw()
            self.update_buttons()

    def undo(self):
        if not self.running and self.history:
            del self.points[self.history.pop():]
            self.update_preview()
            self.draw()
            self.update_buttons()

    def clear(self):
        if not self.running:
            self.points = [(0, 0)]
            self.history.clear()
            self.update_preview()
            self.draw()
            self.update_buttons()

    def pan_start(self, event):
        self.drag_at = (event.x, event.y)

    def pan_move(self, event):
        if self.drag_at:
            self.pan_x += event.x - self.drag_at[0]
            self.pan_y += event.y - self.drag_at[1]
            self.drag_at = (event.x, event.y)
            self.draw()

    def zoom(self, event, linux_direction=None):
        direction = linux_direction if linux_direction is not None else event.delta
        factor = 1.15 if direction > 0 else 1 / 1.15
        new_scale = max(15.0, min(170.0, self.scale * factor))
        # Keep the world coordinate under the mouse fixed while zooming.
        old_scale = self.scale
        cx = self.canvas.winfo_width() / 2
        cy = self.canvas.winfo_height() / 2
        self.pan_x = event.x - cx - (event.x - cx - self.pan_x) * new_scale / old_scale
        self.pan_y = event.y - cy - (event.y - cy - self.pan_y) * new_scale / old_scale
        self.scale = new_scale
        self.draw()

    def read_calibration(self):
        try:
            return int(self.meter_var.get()), int(self.turn_var.get())
        except ValueError:
            raise ValueError("Calibration values must be whole numbers.")

    def update_preview(self):
        x, y = self.points[-1]
        self.position_var.set(f"End point: ({x * GRID_METERS:.1f}, "
                              f"{y * GRID_METERS:.1f}) m")
        try:
            commands = build_commands(self.points, *self.read_calibration())
            preview = "\n".join(commands) if commands else "Click on the grid to draw a path."
        except ValueError as exc:
            preview = str(exc)
        self.preview.configure(state="normal")
        self.preview.delete("1.0", "end")
        self.preview.insert("1.0", preview)
        self.preview.configure(state="disabled")

    def test_connection(self):
        self.start_commands(["S"], test=True)

    def run_path(self):
        try:
            commands = build_commands(self.points, *self.read_calibration())
        except ValueError as exc:
            messagebox.showerror("Calibration", str(exc))
            return
        if not commands:
            messagebox.showinfo("Path", "Click on the grid to draw a path first.")
            return
        self.start_commands(commands, test=False)

    def start_commands(self, commands, test):
        if self.ser is None or self.running:
            return
        self.stop_event.clear()
        with self.serial_lock:
            self.ser.reset_input_buffer()
        self.running = True
        self.status_var.set("Testing Bluetooth..." if test else "Running path...")
        self.update_buttons()
        thread = threading.Thread(target=self.command_worker,
                                  args=(commands, test), daemon=True)
        thread.start()

    def command_worker(self, commands, test):
        try:
            for index, command in enumerate(commands, 1):
                if self.stop_event.is_set():
                    self.events.put(("stopped", None))
                    return
                with self.serial_lock:
                    if self.stop_event.is_set() or self.ser is None:
                        self.events.put(("stopped", None))
                        return
                    port = self.ser
                    port.write((command + "\n").encode("ascii"))
                self.events.put(("status", f"{index}/{len(commands)}: {command}"))

                deadline = time.monotonic() + 8.0
                while time.monotonic() < deadline:
                    if self.stop_event.is_set():
                        self.events.put(("stopped", None))
                        return
                    reply = port.readline().decode("ascii", errors="replace").strip()
                    if reply == "DONE":
                        break
                    if reply in ("BUSY", "ERR"):
                        raise RuntimeError(f"STM32 replied {reply} to {command}")
                else:
                    raise TimeoutError(f"No DONE received for {command}. Check Bluetooth and firmware.")

            self.events.put(("finished", "Bluetooth test successful: DONE"
                             if test else "Path completed: all commands received DONE"))
        except (serial.SerialException, OSError, RuntimeError, TimeoutError) as exc:
            self.stop_event.set()
            try:
                with self.serial_lock:
                    if self.ser is not None and self.ser.is_open:
                        self.ser.write(b"S\n")
            except (serial.SerialException, OSError):
                pass
            self.events.put(("error", str(exc)))

    def stop_robot(self):
        self.stop_event.set()
        try:
            with self.serial_lock:
                if self.ser is not None and self.ser.is_open:
                    self.ser.write(b"S\n")
            self.status_var.set("STOP sent. Disconnect motor power if necessary.")
        except (serial.SerialException, OSError) as exc:
            self.status_var.set(f"STOP failed: {exc}. Disconnect motor battery!")

    def process_events(self):
        while True:
            try:
                kind, value = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == "status":
                self.status_var.set(value)
            else:
                self.running = False
                if kind == "stopped":
                    self.status_var.set("Stopped; reset robot to origin before rerunning.")
                elif kind == "error":
                    self.status_var.set(f"Error: {value}")
                    messagebox.showerror("Robot communication", value)
                else:
                    self.status_var.set(value)
                self.update_buttons()
        self.root.after(50, self.process_events)

    def close(self):
        self.stop_event.set()
        if self.ser is not None:
            try:
                with self.serial_lock:
                    self.ser.write(b"S\n")
                    self.ser.close()
            except (serial.SerialException, OSError):
                pass
        self.root.destroy()


if __name__ == "__main__":
    window = tk.Tk()
    RobotApp(window)
    window.mainloop()
