# Webots + WPILib Sim Integration (Tank Drive Robot)

## Goal
Use WPILib simulation (no RoboRIO) to control a Webots tank robot via Python sockets.

## Components
- `robot.py` – standard WPILib robot code
- `sim_phoenix5/talon.py` – Phoenix5 shim with real/sim split
- `webots_bridge.py` – Webots controller that receives set() values and moves motors
- Uses TCP sockets to send left/right motor power as 2 floats

## Protocol
- WPILib sim sends 2 floats every frame: [left_speed, right_speed]
- Webots controller receives and applies to motors
