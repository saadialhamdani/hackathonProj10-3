"""UNO Q robot: touch-grid path planner + TCP -> Bridge controller (App Lab).

Two ways to drive the robot, both ending up at the same Bridge RPCs:

1. CyberTire web UI on port 7000. Open http://<UNO_Q_IP>:7000 on a phone or
   laptop, tap the Cartesian plane (-5,-5 to 5,5 metres) to drop waypoints,
   press Run. The car starts at the origin (0, 0) facing along +y and visits
   the waypoints in order, turning in place and then driving straight to each.

2. Laptop TCP client on port 8765, one newline-terminated command at a time:
     F,200    forward 200 ms
     L,500    turn left 500 ms
     R,500    turn right 500 ms
     S        stop immediately
   A completed movement or stop gets DONE\n; invalid commands get ERR\n.

The motor sketch independently enforces the 5000 ms per-movement limit.
Only expose these ports to a trusted LOCAL Wi-Fi network. No authentication/TLS.
"""

import math
import socket
import threading
import time

from arduino.app_utils import App, Bridge
from arduino.app_bricks.web_ui import WebUI

# ---------------------------------------------------------------------------
# Grid + calibration. Measure these two on the floor, they are the whole trick.
# ---------------------------------------------------------------------------

# Four-quadrant Cartesian plane in metres: the car starts at the origin facing
# along +y, and waypoints snap to a CELL_M lattice.
GRID_MIN_M = -5.0           # the plane spans GRID_MIN_M..GRID_MAX_M on both axes
GRID_MAX_M = 5.0
CELL_M = 0.5                # waypoint lattice spacing, in metres

MS_PER_METRE = 3230         # forward ms to cover one metre at DRIVE_PERCENT
MS_PER_90_DEG = 290         # turn ms to rotate 90 degrees at TURN_PERCENT

# Break every leg into axis-aligned moves so corners come out square. A
# diagonal leg needs an arbitrary turn angle, and the turn model is least
# accurate there: the fixed kick and brake cost dominates odd angles, so the
# car under-rotates, drives off-line, and the error carries into later legs.
# With this on, every turn is a calibrated 90 or 180 degrees.
SQUARE_CORNERS = True

# Mirror of DRIVE_PERCENT / TURN_PERCENT in sketch.ino, so the calibration
# panel tests at the same power the path runner actually uses. Keep in sync.
DRIVE_PERCENT = 60
TURN_PERCENT = 90

# Mirror of OBSTACLE_STOP_CM in sketch.ino, shown in the UI. The cut-out itself
# runs on the MCU so it does not depend on Python being responsive.
OBSTACLE_STOP_CM = 20

# Mirror of leftReversed / rightReversed in sketch.ino. The MCU boots with the
# sketch's values, so keep these equal to them.
invert = {"left": True, "right": False}

MAX_POINTS = 32
MAX_MS = 5000               # must match MAX_MOVE_MS in the sketch
TEST_MAX_MS = 20000         # calibration moves may span several MAX_MS chunks
SETTLE_S = 0.25             # pause between segments so the chassis stops rocking

# ---------------------------------------------------------------------------
# TCP server (laptop path)
# ---------------------------------------------------------------------------

HOST = "0.0.0.0"
PORT = 8765

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((HOST, PORT))
server.listen(1)
server.setblocking(False)

client = None
input_buffer = b""
moving = False
next_status_check = 0.0

# ---------------------------------------------------------------------------
# Shared motion state
# ---------------------------------------------------------------------------

bridge_lock = threading.Lock()      # serialises Bridge calls across threads
path_stop = threading.Event()
path_thread = None

car = {
    "x": 0.0,          # metres along +x
    "y": 0.0,          # metres along +y
    "heading": 0.0,    # degrees, 0 = along +y, positive = clockwise
    "index": 0,        # waypoints completed
    "total": 0,
    "running": False,
    "obstacle": False, # last path was cut short by the rangefinder
    "message": "Idle at (0, 0) facing +y",
}


def bridge_command(action, duration_ms):
    with bridge_lock:
        return int(Bridge.call("robot_command", int(action), int(duration_ms)))


def bridge_drive(left_percent, right_percent, duration_ms):
    with bridge_lock:
        return int(Bridge.call("robot_drive", int(left_percent),
                               int(right_percent), int(duration_ms)))


def bridge_status():
    with bridge_lock:
        return int(Bridge.call("robot_status"))


def bridge_distance():
    """Rangefinder reading in cm, or None when nothing is in range."""
    try:
        with bridge_lock:
            value = int(Bridge.call("robot_distance"))
    except Exception as exc:
        print("Distance read failed:", exc)
        return None
    return value if value > 0 else None


def bridge_obstacle():
    try:
        with bridge_lock:
            return int(Bridge.call("robot_obstacle")) == 1
    except Exception as exc:
        print("Obstacle read failed:", exc)
        return False


def stop_on_mcu():
    global moving
    moving = False
    try:
        bridge_command(0, 0)
    except Exception as exc:
        print("Could not send STOP to MCU:", exc)


# ---------------------------------------------------------------------------
# Path execution
# ---------------------------------------------------------------------------


def wait_until_stopped(duration_ms):
    """Block until the MCU reports stopped. False if stopped early or timed out."""
    deadline = time.monotonic() + (duration_ms / 1000.0) + 2.0
    while time.monotonic() < deadline:
        if path_stop.is_set():
            return False
        try:
            if bridge_status() == 0:
                return True
        except Exception as exc:
            print("Movement status failed:", exc)
            return False
        time.sleep(0.04)
    print("Timed out waiting for the MCU to finish a move")
    return False


def run_segment(action, duration_ms):
    """One turn or forward burst, split into <= MAX_MS chunks. True if complete."""
    remaining = int(round(duration_ms))
    while remaining > 0:
        if path_stop.is_set():
            return False
        chunk = min(remaining, MAX_MS)
        if chunk < 1:
            return True
        try:
            result = bridge_command(action, chunk)
        except Exception as exc:
            print("Motor command failed:", exc)
            return False
        if result != 1:
            print("MCU refused command:", action, chunk, "->", result)
            return False
        if not wait_until_stopped(chunk):
            return False
        remaining -= chunk
        time.sleep(SETTLE_S)
    return True


def squarify(cx, cy, points):
    """Expand waypoints into axis-aligned legs, y first then x.

    Returns [(x, y, waypoint_index_or_None)] so the UI can still count the
    waypoints you actually tapped rather than the intermediate corners.
    """
    legs = []
    for i, (tx, ty) in enumerate(points):
        if abs(ty - cy) > 1e-6:
            legs.append([cx, ty, None])
            cy = ty
        if abs(tx - cx) > 1e-6:
            legs.append([tx, ty, None])
            cx = tx
        if legs and legs[-1][2] is None:
            legs[-1][2] = i          # the leg that lands on this waypoint
        else:
            legs.append([tx, ty, i])  # already there: a no-op leg
    return [tuple(leg) for leg in legs]


def drive_path(points, square):
    """Turn-then-go along each leg. Runs on its own thread."""
    if square:
        legs = squarify(car["x"], car["y"], points)
    else:
        legs = [(x, y, i) for i, (x, y) in enumerate(points)]

    car["total"] = len(points)
    car["index"] = 0
    car["running"] = True
    car["obstacle"] = False
    car["message"] = "Running"

    try:
        for target_x, target_y, waypoint in legs:
            if path_stop.is_set():
                break

            dx = target_x - car["x"]
            dy = target_y - car["y"]
            distance = math.hypot(dx, dy)
            if distance < 0.02:
                if waypoint is not None:
                    car["index"] = waypoint + 1
                continue

            # 0 degrees is up the grid (+y), angles increase clockwise.
            target_heading = math.degrees(math.atan2(dx, dy))
            turn = (target_heading - car["heading"] + 180.0) % 360.0 - 180.0

            if abs(turn) > 1.0:
                action = 3 if turn > 0 else 2      # 3 = right, 2 = left
                turn_ms = abs(turn) / 90.0 * MS_PER_90_DEG
                car["message"] = "Turning %s %.0f deg" % (
                    "right" if turn > 0 else "left", abs(turn))
                if not run_segment(action, turn_ms):
                    break
                car["heading"] = (car["heading"] + turn) % 360.0

            car["message"] = "Driving %.2f m" % distance
            if not run_segment(1, distance * MS_PER_METRE):
                break

            # The MCU brakes on its own if something is too close; the path is
            # void from here because we did not travel the full leg.
            if bridge_obstacle():
                near = bridge_distance()
                car["message"] = (
                    "Obstacle at %s cm — path abandoned, press Reset"
                    % (near if near is not None else "?"))
                car["obstacle"] = True
                return

            car["x"] = target_x
            car["y"] = target_y
            if waypoint is not None:
                car["index"] = waypoint + 1

        if path_stop.is_set():
            car["message"] = "Stopped — position is now unknown, press Reset"
        elif car["index"] == car["total"]:
            car["message"] = "Path complete"
        else:
            car["message"] = "Aborted after %d of %d waypoints" % (
                car["index"], car["total"])
    finally:
        try:
            bridge_command(0, 0)
        except Exception:
            pass
        car["running"] = False


# ---------------------------------------------------------------------------
# Web UI API
# ---------------------------------------------------------------------------

ui = WebUI()


def api_config():
    return {
        "min_m": GRID_MIN_M,
        "max_m": GRID_MAX_M,
        "cell_m": CELL_M,
        "ms_per_metre": MS_PER_METRE,
        "ms_per_90_deg": MS_PER_90_DEG,
        "max_points": MAX_POINTS,
        "drive_percent": DRIVE_PERCENT,
        "turn_percent": TURN_PERCENT,
        "max_ms": MAX_MS,
        "test_max_ms": TEST_MAX_MS,
        "invert_left": invert["left"],
        "invert_right": invert["right"],
        "obstacle_stop_cm": OBSTACLE_STOP_CM,
        "square_corners": SQUARE_CORNERS,
    }


def api_state():
    return {
        "running": car["running"],
        "x": round(car["x"], 3),
        "y": round(car["y"], 3),
        "heading": round(car["heading"], 1),
        "index": car["index"],
        "total": car["total"],
        "obstacle": car["obstacle"],
        "distance_cm": bridge_distance(),
        "message": car["message"],
    }


def api_run(payload: dict):
    global path_thread

    if car["running"]:
        return {"error": "A path is already running"}
    if moving:
        return {"error": "The laptop TCP client is driving the robot"}

    raw = payload.get("points") or []
    if not isinstance(raw, list) or not raw:
        return {"error": "No waypoints"}
    if len(raw) > MAX_POINTS:
        return {"error": "Too many waypoints (max %d)" % MAX_POINTS}

    points = []
    for item in raw:
        if not isinstance(item, (list, tuple)) or len(item) != 2:
            return {"error": "Waypoints must be [x, y] pairs"}
        try:
            x, y = float(item[0]), float(item[1])
        except (TypeError, ValueError):
            return {"error": "Waypoints must be numbers"}
        if not GRID_MIN_M <= x <= GRID_MAX_M or not GRID_MIN_M <= y <= GRID_MAX_M:
            return {"error": "Waypoints must be inside %g..%g m"
                             % (GRID_MIN_M, GRID_MAX_M)}
        points.append((x, y))

    square = bool(payload.get("square", SQUARE_CORNERS))

    path_stop.clear()
    path_thread = threading.Thread(
        target=drive_path, args=(points, square), daemon=True)
    path_thread.start()
    return {"ok": True, "waypoints": len(points), "square": square}


def run_raw(left, right, duration_ms, label):
    """Raw per-wheel burst for the calibration panel, split into MAX_MS chunks."""
    car["running"] = True
    car["message"] = label
    remaining = int(duration_ms)
    try:
        while remaining > 0:
            if path_stop.is_set():
                car["message"] = "%s stopped" % label
                return
            chunk = min(remaining, MAX_MS)
            result = bridge_drive(left, right, chunk)
            if result != 1:
                car["message"] = "MCU refused the test (%d)" % result
                return
            if not wait_until_stopped(chunk):
                car["message"] = "%s did not finish" % label
                return
            remaining -= chunk
            if remaining > 0:
                time.sleep(SETTLE_S)
        car["message"] = "%s done — dead reckoning is now stale, press Reset" % label
    except Exception as exc:
        print("Test move failed:", exc)
        car["message"] = "Test failed: %s" % exc
    finally:
        try:
            bridge_command(0, 0)
        except Exception:
            pass
        car["running"] = False


def api_test(payload: dict):
    """Raw wheel test: {left: -100..100, right: -100..100, ms: 1..5000}."""
    global path_thread

    if car["running"]:
        return {"error": "Already moving"}
    if moving:
        return {"error": "The laptop TCP client is driving the robot"}

    try:
        left = int(payload.get("left", 0))
        right = int(payload.get("right", 0))
        duration = int(payload.get("ms", 0))
    except (TypeError, ValueError):
        return {"error": "left, right and ms must be numbers"}

    if not -100 <= left <= 100 or not -100 <= right <= 100:
        return {"error": "Wheel power must be -100..100"}
    if not 1 <= duration <= TEST_MAX_MS:
        return {"error": "ms must be 1..%d" % TEST_MAX_MS}

    label = str(payload.get("label") or "Test move")
    path_stop.clear()
    path_thread = threading.Thread(
        target=run_raw, args=(left, right, duration, label), daemon=True)
    path_thread.start()
    return {"ok": True}


def api_invert(payload: dict):
    """Flip wheel polarity live: {left: bool, right: bool}."""
    if car["running"]:
        return {"error": "Stop the robot first"}

    left = bool(payload.get("left", invert["left"]))
    right = bool(payload.get("right", invert["right"]))

    try:
        with bridge_lock:
            Bridge.call("robot_invert", 1 if left else 0, 1 if right else 0)
    except Exception as exc:
        print("Could not set wheel polarity:", exc)
        return {"error": "MCU did not accept the change: %s" % exc}

    invert["left"] = left
    invert["right"] = right
    car["message"] = "Invert: left=%s right=%s" % (left, right)
    print("Wheel polarity set to left=%s right=%s" % (left, right))
    print("Set leftReversed / rightReversed in sketch.ino to keep this.")
    return {"ok": True, "left": left, "right": right}


def api_tune(payload: dict):
    """Update the two timing constants without restarting the app."""
    global MS_PER_METRE, MS_PER_90_DEG

    if car["running"]:
        return {"error": "Stop the robot first"}

    try:
        metre = int(payload.get("ms_per_metre", MS_PER_METRE))
        turn = int(payload.get("ms_per_90_deg", MS_PER_90_DEG))
    except (TypeError, ValueError):
        return {"error": "Values must be whole numbers of milliseconds"}

    if metre < 1 or not 1 <= turn <= MAX_MS:
        return {"error": "ms per 90 deg must be 1..%d" % MAX_MS}

    MS_PER_METRE = metre
    MS_PER_90_DEG = turn
    car["message"] = "Calibration set: %d ms/m, %d ms/90 deg" % (metre, turn)
    print("Calibration updated:", metre, "ms/m,", turn, "ms/90deg")
    print("Edit MS_PER_METRE / MS_PER_90_DEG in main.py to make this permanent.")
    return {"ok": True, "ms_per_metre": metre, "ms_per_90_deg": turn}


def api_stop():
    path_stop.set()
    stop_on_mcu()
    car["message"] = "Stopped"
    return {"ok": True}


def api_reset():
    """Pick the car up, put it back on the origin facing up, then call this."""
    if car["running"]:
        return {"error": "Stop the path first"}
    car["x"] = 0.0
    car["y"] = 0.0
    car["heading"] = 0.0
    car["index"] = 0
    car["total"] = 0
    car["obstacle"] = False
    car["message"] = "Idle at (0, 0) facing +y"
    return {"ok": True}


ui.expose_api("GET", "/config", api_config)
ui.expose_api("GET", "/state", api_state)
ui.expose_api("POST", "/run", api_run)
ui.expose_api("POST", "/test", api_test)
ui.expose_api("POST", "/invert", api_invert)
ui.expose_api("POST", "/tune", api_tune)
ui.expose_api("POST", "/stop", api_stop)
ui.expose_api("POST", "/reset", api_reset)


# ---------------------------------------------------------------------------
# TCP command handling
# ---------------------------------------------------------------------------


def close_client():
    global client, input_buffer
    # Stop on disconnection; MCU also has its own movement timeout.
    stop_on_mcu()
    if client is not None:
        try:
            client.close()
        except OSError:
            pass
    client = None
    input_buffer = b""


def reply(message):
    if client is not None:
        client.sendall((message + "\n").encode("ascii"))


def process_command(line):
    global moving, next_status_check
    try:
        command = line.decode("ascii").strip().upper()
    except UnicodeDecodeError:
        reply("ERR")
        return

    if command == "S":
        path_stop.set()
        try:
            if bridge_command(0, 0) != 1:
                raise RuntimeError("Stop was not accepted")
            moving = False
            reply("DONE")
        except Exception as exc:
            print("STOP failed:", exc)
            reply("ERR")
        return

    if moving or car["running"]:
        reply("BUSY")
        return

    parts = command.split(",")
    if len(parts) != 2 or parts[0] not in ("F", "L", "R"):
        reply("ERR")
        return
    if not parts[1].isascii() or not parts[1].isdigit():
        reply("ERR")
        return

    duration = int(parts[1])
    if not 1 <= duration <= MAX_MS:
        reply("ERR")
        return

    action = {"F": 1, "L": 2, "R": 3}[parts[0]]
    try:
        result = bridge_command(action, duration)
        if result == 1:
            moving = True
            next_status_check = time.monotonic() + 0.04
        elif result == 0:
            reply("BUSY")
        else:
            reply("ERR")
    except Exception as exc:
        print("Motor command failed:", exc)
        stop_on_mcu()
        reply("ERR")


def loop():
    global client, input_buffer, moving, next_status_check

    if client is None:
        try:
            client, address = server.accept()
            client.setblocking(False)
            input_buffer = b""
            print("Laptop connected:", address)
        except BlockingIOError:
            time.sleep(0.02)
            return

    if moving and time.monotonic() >= next_status_check:
        try:
            status = bridge_status()
            next_status_check = time.monotonic() + 0.06
            if status == 0:
                moving = False
                reply("DONE")
        except Exception as exc:
            print("Movement status failed:", exc)
            close_client()
            time.sleep(0.02)
            return

    try:
        data = client.recv(256)
        if not data:
            print("Laptop disconnected")
            close_client()
            return
        input_buffer += data

        if len(input_buffer) > 256:
            print("Oversized message rejected")
            close_client()
            return

        while b"\n" in input_buffer:
            raw_line, input_buffer = input_buffer.split(b"\n", 1)
            process_command(raw_line.rstrip(b"\r"))

    except BlockingIOError:
        pass
    except (OSError, ConnectionError) as exc:
        print("Connection lost:", exc)
        close_client()

    time.sleep(0.02)


App.run(user_loop=loop)
