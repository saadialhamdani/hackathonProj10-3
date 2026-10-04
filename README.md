Draw-a-Path Car

Tap waypoints on a grid in your browser, press Run, and the car drives the path on its own. Built at a 24-hour hackathon in Dearborn (theme: Conjure Reality).

The car has no wheel encoders. It uses timed moves: we measured how long it takes to drive 1 m and to turn 90°, and the planner turns each leg of the path into timed motor commands.

From STM32 to Arduino UNO Q

We started on an STM32F401RE with bare-metal C firmware, an HC-05 Bluetooth module, and a desktop Python app (main.py) that sent commands over Bluetooth serial.

We switched to the Arduino UNO Q because it has built-in Wi-Fi and runs Linux, so we could drop the Bluetooth module and run the path planner on the car itself. Now the car is controlled from any browser on the same network. The motor driver wiring and the F,<ms> / L,<ms> / R,<ms> / S command format carried over unchanged.

Files
File	What it is
sketch.ino	UNO Q motor control (TB6612FNG)
main2.py	UNO Q app: web UI, path planner, TCP server
main.py, *.c, *.h, stm32/	Original STM32 + Bluetooth version
Hardware

Arduino UNO Q · TB6612FNG motor driver · 2WD chassis · AA battery pack (motors) · USB-C power bank (UNO Q)

Run it
Upload sketch.ino and main2.py in Arduino App Lab.
Open http://<UNO_Q_IP>:7000 on a device on the same Wi-Fi.
Place the car at the origin, tap waypoints, and press Run.

Tune MS_PER_METRE and MS_PER_90_DEG in main2.py for your floor.
