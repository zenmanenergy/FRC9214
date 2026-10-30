# Setup step 0: Your robot's settings

## What this step is for

The code in this project works for any robot built from REV EasySwerve modules. But every robot is wired a little differently. **robot_settings.py** is the one file that tells the code how YOUR robot is built: which motor controller is which, where each sensor is plugged in, and how big the robot is.

In this step you:

1. Learn the parts of the robot and the tools on the laptop.
2. Fill in robot_settings.py, one line at a time, by looking at your robot.
3. Deploy (send the code to the robot) for the first time.
4. Read the robot's report, which prints your settings back in plain English and points out anything that looks wrong.

Nothing moves in this step.

## The parts of the robot

| Part | What it is |
|---|---|
| roboRIO | The robot's computer. Your code runs on it. It has rows of ports along its edges: DIO, PWM, CAN, USB, and the MXP port on top. |
| Radio | The robot's Wi-Fi. Your laptop connects to it to talk to the roboRIO. |
| Driver Station | The program on the laptop that enables and disables the robot and reads the controllers. |
| Swerve module | One corner of the drivetrain: a wheel that can spin AND turn to point any direction. Your robot has four: front left, front right, back left, back right. |
| NEO 2.0 | A motor. Each swerve module has two: the drive motor spins the wheel, the steering motor turns it. That's 8 motors. |
| SPARK MAX | A motor controller: the box between the battery power and one motor. It does what the roboRIO tells it. One per motor, so 8 of them. |
| CAN bus | The yellow and green wires that run from the roboRIO to every SPARK MAX, one after another in a chain. The roboRIO talks to all 8 SPARK MAXes over this one cable. |
| CAN ID | Each SPARK MAX's number on the CAN bus, like an address. Every one must be different, so the roboRIO knows who is who. |
| Through Bore Encoder | A sensor on each swerve module that measures which way the wheel points. Its cable plugs into one DIO port on the roboRIO. |
| DIO port | "Digital input/output" ports on the roboRIO, numbered 0 to 9 and printed on the case. Each steering encoder uses one. |
| navX2 | The gyro: it measures which way the whole robot is facing. It plugs onto the MXP port on top of the roboRIO (or into a USB port). |

Looking at the robot from above, with its front pointing away from you:

```
        FRONT
  front left   front right
  (FRONT_LEFT) (FRONT_RIGHT)

  back left    back right
  (BACK_LEFT)  (BACK_RIGHT)
        BACK
```

Left and right are the robot's left and right, not yours if you're facing it.

## The tools on the laptop

| Tool | What it's for | Where it comes from |
|---|---|---|
| VS Code | Where you read and edit the code. | code.visualstudio.com |
| Python and RobotPy | Python is the programming language; RobotPy lets Python run on an FRC robot. | Root README.md, Day 1, steps 1 to 3 |
| FRC Game Tools | Installs the Driver Station. | NI's FRC Game Tools download page |
| REV Hardware Client | Reads and sets each SPARK MAX's CAN ID over a USB cable. | REV Robotics' website |

Your coach or a mentor does two things once each season: setting up the roboRIO for your team number (with the roboRIO Imaging Tool from FRC Game Tools) and programming the radio.

## How to read robot_settings.py

Open robot_settings.py in VS Code. It's Python, but you only need four rules:

- **name = value** sets a setting. Change what's after the = sign. Leave the name alone.
- A line starting with **#** is a comment: a note for people. The robot ignores it.
- **True** and **False** (capital T and F) are yes and no.
- Text goes in quote marks: "rio", "mxp". Numbers don't: 9, 0.6858.

A swerve module line looks like this:

```python
FRONT_LEFT = Module(drive_can=9, turn_can=8, encoder_dio=0, drive_motor_bottom=True, turn_motor_bottom=True)
```

It's one line with five settings for the front left module, separated by commas. Change the numbers and the True/False; keep the commas and the brackets.

All lengths are in **meters**. A tape measure in centimeters works: 60 cm is 0.60 m.

## Fill it in, line by line

Print the worksheet below, fill it in at the robot, then copy it into robot_settings.py.

### drive_can and turn_can (each module)

The CAN ID of the SPARK MAX for that module's drive motor (the one that spins the wheel) and its steering motor (the one that turns it).

How to find them:

1. Follow the power wires from each motor to its SPARK MAX. Put a label on every SPARK MAX: "FL drive", "FL steer", and so on.
2. Install REV Hardware Client. Plug a USB-C cable from the laptop into one SPARK MAX (the robot can be on or off).
3. REV Hardware Client shows that SPARK MAX's CAN ID. Write it on its label and on the worksheet.
4. Do all 8.

Rules: every CAN ID must be different, between 1 and 62. If two are the same, or one is 0, change it in REV Hardware Client and write the new number down. A simple plan: front left 1 and 2, front right 3 and 4, back left 5 and 6, back right 7 and 8, with the drive motor first.

### encoder_dio (each module)

The roboRIO DIO port that module's steering encoder is plugged into. Follow the encoder's cable from the module to the roboRIO and read the port number printed next to it (0 to 9).

Setup step 2 checks these for you by having you turn each wheel by hand.

### drive_motor_bottom and turn_motor_bottom (each module)

On an EasySwerve module, each motor can be mounted under the module's plate or on top of it. That changes which way the motor has to spin. Look at each module from the side:

- drive_motor_bottom: True if the drive motor hangs below the plate, False if it sits on top.
- turn_motor_bottom: the same, for the steering motor.

Not sure? Leave them True. Setup steps 3 and 6 spin each motor and tell you if one needs changing.

### encoder_reversed (each module, optional)

Leave this out at first. If an encoder counts backwards, setup step 3 tells you to add encoder_reversed=True to that module's line.

### STEERING_ENCODERS

Where the steering encoders plug in: "rio" if their cables go to the roboRIO's DIO ports (your robot), "spark" if they plug into the steering SPARK MAX instead.

### ENCODER_VERSION

Which Through Bore Encoder you have: 1 for the original (part number REV-11-1271) or 2 for the V2 (REV-11-3174). The part number is on the box and on REV's order page.

### STEERING_ZEROS_DEG

Leave it as None. Setup step 4 measures it and tells you what to put here.

### TRACK_WIDTH_M and WHEEL_BASE_M

The size of the robot, measured from the CENTER of one wheel to the CENTER of the other (not the frame, not the bumpers):

- TRACK_WIDTH_M: left wheels to right wheels.
- WHEEL_BASE_M: front wheels to back wheels.

Measure in centimeters and divide by 100: 58.4 cm is 0.584.

### WHEEL_DIAMETER_M

Leave it at 0.1016 (new EasySwerve wheels). Setup step 10 measures the real size later, because treads wear down.

### MAX_SPEED_MPS and MAX_TURN_DEG_PER_S

How fast the robot goes with the stick pushed all the way, in meters per second, and how fast it spins, in degrees per second. The wheels can do about 4.8 m/s, which is fast and dangerous in a small room. **For the first drives, set MAX_SPEED_MPS = 1.5** and MAX_TURN_DEG_PER_S = 180. Raise them later. (The examples' READMEs work out distances with 4.5; at 1.5 every auto goes a third as far, which is fine.)

### CONTROLLER_PORT

Which slot the driver's Xbox controller is in on the Driver Station's USB tab (the tab with the USB symbol). Usually 0.

### GYRO

Where the navX2 is plugged in: "mxp" if it sits on the MXP port on top of the roboRIO, "usb" if it's on a USB cable.

### VISION_ENABLED and CAMERAS

Leave VISION_ENABLED = False. The cameras come much later (docs/VISION.md).

## Worksheet

Print this and fill it in at the robot.

| Module | drive_can | turn_can | encoder_dio | drive motor under the plate? | steering motor under the plate? |
|---|---|---|---|---|---|
| FRONT_LEFT | | | | | |
| FRONT_RIGHT | | | | | |
| BACK_LEFT | | | | | |
| BACK_RIGHT | | | | | |

| Setting | Value |
|---|---|
| ENCODER_VERSION (1 or 2) | |
| TRACK_WIDTH_M (left to right wheel centers) | |
| WHEEL_BASE_M (front to back wheel centers) | |
| GYRO ("mxp" or "usb") | |

## Deploy for the first time

"Deploy" means copying your code onto the roboRIO so the robot runs it.

1. Copy this folder's robot.py over robot.py in the project folder. In VS Code: right-click this folder's robot.py, Copy, then right-click the project folder (the one with robot_settings.py in it), Paste, and choose Replace.
2. While the laptop has internet, open the VS Code terminal (Ctrl+`, the key above Tab) and run this once. It downloads what the roboRIO needs:

```
py -m robotpy sync
```

3. Turn the robot on and wait about a minute. Connect the laptop to the robot's radio Wi-Fi (its name starts with your team number), or plug a USB cable from the laptop into the roboRIO.
4. In the VS Code terminal:

```
py -m robotpy deploy
```

5. The first time, it asks for your team number. Type it and press Enter.
6. Wait for it to finish. The terminal then keeps showing what the robot prints (press Ctrl+C to stop watching). The Driver Station shows the same messages.
7. Read the report. It starts with "Your code is on the robot!", then lists every setting in plain English, then "things to check". Fix each one in robot_settings.py and deploy again, until it says "Nothing looks wrong".

Every time you change robot.py or robot_settings.py, deploy again. The robot only runs what was last deployed.

## The Driver Station in one minute

- **Mode buttons** (left side): Teleoperated (a driver uses a controller), Autonomous (the robot drives itself), Test (used for tuning, setup step 8).
- **Enable** starts the robot in that mode. **Disable** stops it.
- **Enter** disables. **Space bar** is the emergency stop: the robot stays stopped until it's restarted. Whoever is near the laptop keeps a hand near both, every time the robot is enabled.
- The **USB tab** shows the controllers and their slot numbers.
- The lights at the top: Communications, Robot Code and Joysticks should all be green before you enable.

## If something goes wrong

| What you see | What it means |
|---|---|
| SyntaxError with a line number | A typo in that line of robot_settings.py: a missing comma or bracket, or true instead of True. Compare it with the example line above. |
| NameError | A setting's name was changed or misspelled. Names must stay exactly as they were. |
| Deploy can't find the robot | The laptop isn't connected to the robot's radio (or USB), the robot isn't on, or the team number is wrong. |
| Robot Code light stays red | The code crashed on the robot. Read the red error in the Driver Station or the terminal. |

## The code to look at

This step's robot.py doesn't build a robot: it only reads robot_settings.py and prints it.

```python
import robot_settings as S
```

This loads robot_settings.py, so S.TRACK_WIDTH_M is the number you typed for TRACK_WIDTH_M.

```python
            if value > 5:
                notes.append(f"{label} = {value} looks like centimeters. It's in meters: 60 cm is 0.60.")
```

Each check is a simple question about a setting. If the answer looks wrong, it adds a note to the list. At the end, the notes are printed with numbers.

## Try this

1. Type TRACK_WIDTH_M = 60 (centimeters by mistake), deploy, and read the report. Then fix it.
2. Give two SPARK MAXes the same CAN ID in robot_settings.py. What does the report say?
3. Delete a comma from one of the Module lines and deploy. Read the error, find the line, fix it.

Next: [00_01_checkup](../00_01_checkup/README.md)

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
