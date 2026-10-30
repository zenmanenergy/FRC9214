# Examples

Each folder has a robot.py that shows one idea, and a README.md that explains it: what it does, the code to look at, the key ideas, and things to try. The numbers are the order to try them in; each one builds on the ones before.

To try one:

1. Copy the example's robot.py over robot.py in the project folder (the one next to robot_settings.py). In VS Code: right-click the example's robot.py, Copy, then right-click the project folder, Paste, and choose Replace.
2. Run the simulator: py -m robotpy sim
3. Read the example's README.md while you try it.

To go back to the start: copy examples/01_basic/robot.py over robot.py.

Every example's auto uses gentle speeds (about 2 meters per move), so they're safe to try on the real robot too, with room to move.

## Start here

| # | Folder | What it shows |
|---|---|---|
| 01 | [01_basic](01_basic/README.md) | the starting robot: drive, 2 buttons, PathPlanner autos |

## Simple autos (click Autonomous)

| # | Folder | What it shows |
|---|---|---|
| 02 | [02_driveforward](02_driveforward/README.md) | drive forward for 2 seconds |
| 03 | [03_drivebackward](03_drivebackward/README.md) | drive backward |
| 04 | [04_strafe](04_strafe/README.md) | slide sideways |
| 05 | [05_spin](05_spin/README.md) | spin in place |
| 06 | [06_diagonal](06_diagonal/README.md) | drive at an angle |
| 07 | [07_arc](07_arc/README.md) | drive in a curve |
| 08 | [08_square](08_square/README.md) | a square: commands one after another |
| 09 | [09_waitthendrive](09_waitthendrive/README.md) | wait, then drive |
| 10 | [10_printmessages](10_printmessages/README.md) | print messages while the auto runs |
| 11 | [11_backandforth](11_backandforth/README.md) | repeat a move |
| 12 | [12_drivedistance](12_drivedistance/README.md) | drive exactly 2 meters |
| 13 | [13_turntoangle](13_turntoangle/README.md) | turn to face 90 degrees |
| 14 | [14_lshape](14_lshape/README.md) | drive, turn a corner, drive |
| 15 | [15_stopandlock](15_stopandlock/README.md) | drive, then lock the wheels |
| 16 | [16_autochooser](16_autochooser/README.md) | pick an auto from a dashboard list |

## PathPlanner autos

| # | Folder | What it shows |
|---|---|---|
| 17 | [17_pathplannerauto](17_pathplannerauto/README.md) | run one auto from the PathPlanner app |
| 18 | [18_drivepath](18_drivepath/README.md) | drive one path |
| 19 | [19_paththenturn](19_paththenturn/README.md) | a path, then a turn |
| 20 | [20_namedcommands](20_namedcommands/README.md) | autos that run your commands by name |

## Buttons and driving (click Teleoperated, needs a controller)

| # | Folder | What it shows |
|---|---|---|
| 21 | [21_buttons](21_buttons/README.md) | more buttons: stop, lock, face a direction |
| 22 | [22_dpadturn](22_dpadturn/README.md) | point the robot with the D-pad |
| 23 | [23_robotrelative](23_robotrelative/README.md) | drive like a remote-control car |
| 24 | [24_slowmode](24_slowmode/README.md) | hold a trigger to drive slowly |
| 25 | [25_drivetopoint](25_drivetopoint/README.md) | hold A: drive itself to a spot |
| 26 | [26_pathbutton](26_pathbutton/README.md) | hold A: line up and drive a path |
| 27 | [27_rumble](27_rumble/README.md) | make the controller rumble |
| 28 | [28_twocontrollers](28_twocontrollers/README.md) | driver and operator controllers |

## Dashboard

| # | Folder | What it shows |
|---|---|---|
| 29 | [29_dashboard](29_dashboard/README.md) | show the robot's position and heading |

## Mechanisms

| # | Folder | What it shows |
|---|---|---|
| 30 | [30_motor](30_motor/README.md) | spin one motor with a button |
| 31 | [31_intake](31_intake/README.md) | an intake: a subsystem, buttons, and an auto |

## Calibration (on the real robot, before driving it hard)

| # | Folder | What it shows |
|---|---|---|
| 32 | [32_zerosteering](32_zerosteering/README.md) | teach each wheel where straight ahead is |
| 33 | [33_gyrocheck](33_gyrocheck/README.md) | check the gyro: calibrating, drift, turning test |
| 34 | [34_wheeldiameter](34_wheeldiameter/README.md) | measure the real wheel size so distances are right |
