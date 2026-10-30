# Examples

Each folder has a robot.py that shows one idea.

To try one:

1. Copy the example's robot.py over robot.py in the project folder (the one next to robot_settings.py). In VS Code: right-click the example's robot.py, Copy, then right-click the project folder, Paste, and choose Replace.
2. Run the simulator: py -m robotpy sim
3. Do what the example's "Try it" line says.

To go back to the start: copy examples/basic/robot.py over robot.py.

Every example's auto uses gentle speeds (about 2 meters per move), so they're safe to try on the real robot too, with room to move.

## Start here

| # | Folder | What it shows |
|---|---|---|
| 1 | basic | the starting robot: drive, 2 buttons, PathPlanner autos |

## Simple autos (click Autonomous)

| # | Folder | What it shows |
|---|---|---|
| 2 | driveforward | drive forward for 2 seconds |
| 3 | drivebackward | drive backward |
| 4 | strafe | slide sideways |
| 5 | spin | spin in place |
| 6 | diagonal | drive at an angle |
| 7 | arc | drive in a curve |
| 8 | square | a square: commands one after another |
| 9 | waitthendrive | wait, then drive |
| 10 | printmessages | print messages while the auto runs |
| 11 | backandforth | repeat a move |
| 12 | drivedistance | drive exactly 2 meters |
| 13 | turntoangle | turn to face 90 degrees |
| 14 | lshape | drive, turn a corner, drive |
| 15 | stopandlock | drive, then lock the wheels |
| 16 | autochooser | pick an auto from a dashboard list |

## PathPlanner autos

| # | Folder | What it shows |
|---|---|---|
| 17 | pathplannerauto | run one auto from the PathPlanner app |
| 18 | drivepath | drive one path |
| 19 | paththenturn | a path, then a turn |
| 20 | namedcommands | autos that run your commands by name |

## Buttons and driving (click Teleoperated, needs a controller)

| # | Folder | What it shows |
|---|---|---|
| 21 | buttons | more buttons: stop, lock, face a direction |
| 22 | dpadturn | point the robot with the D-pad |
| 23 | robotrelative | drive like a remote-control car |
| 24 | slowmode | hold a trigger to drive slowly |
| 25 | drivetopoint | hold A: drive itself to a spot |
| 26 | pathbutton | hold A: line up and drive a path |
| 27 | rumble | make the controller rumble |
| 28 | twocontrollers | driver and operator controllers |

## Dashboard

| # | Folder | What it shows |
|---|---|---|
| 29 | dashboard | show the robot's position and heading |

## Mechanisms

| # | Folder | What it shows |
|---|---|---|
| 30 | motor | spin one motor with a button |
| 31 | intake | an intake: a subsystem, buttons, and an auto |
