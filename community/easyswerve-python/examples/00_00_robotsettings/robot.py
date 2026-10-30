#!/usr/bin/env python3
"""
SETUP STEP 0: Your robot's settings (nothing moves)

robot_settings.py tells the code how YOUR robot is built and wired: which
motor controller is which, where each sensor is plugged in, and how big the
robot is. The README in this folder explains every line and how to find each
value on your robot.

This robot.py reads robot_settings.py and prints it back in plain English,
then points out anything that looks wrong: example numbers you haven't
changed yet, the same number used twice, centimeters typed where meters go.
It is also your first deploy: if you can read its message, your code is on
the robot.

On the robot: fill in robot_settings.py, deploy, leave the robot DISABLED,
and read the console. Fix anything it lists, deploy again, and repeat.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy
"""

import commands2
import wpilib

import robot_settings as S

# The numbers robot_settings.py came with. If yours still match, you probably
# haven't checked them yet.
EXAMPLE_CAN_IDS = {"FRONT_LEFT": (9, 8), "FRONT_RIGHT": (11, 10), "BACK_LEFT": (15, 14), "BACK_RIGHT": (13, 12)}
EXAMPLE_SIZE_M = 0.6858

MODULES = [
    ("FRONT_LEFT", "front left"),
    ("FRONT_RIGHT", "front right"),
    ("BACK_LEFT", "back left"),
    ("BACK_RIGHT", "back right"),
]


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Runs once when the robot turns on."""
        # Nothing to build: this step only reads robot_settings.py.
        self.lastReport = None
        self.startTime = wpilib.Timer.getFPGATimestamp()

    def robotPeriodic(self) -> None:
        """Runs 50 times a second. Print the report now and then."""
        now = wpilib.Timer.getFPGATimestamp()
        if now - self.startTime < 2.0:
            return  # give the console a moment to connect after deploying
        if self.lastReport is not None and now - self.lastReport < 15.0:
            return
        self.lastReport = now
        self.report()

    def report(self) -> None:
        notes = []  # things to check, in plain English

        print("==== Your code is on the robot! This is what robot_settings.py says: ====")

        # ---- the four swerve modules
        cans = {}
        dios = {}
        for name, words in MODULES:
            m = getattr(S, name)
            print(f"{words.upper()} module:")
            print(f"    the motor that spins the wheel  = SPARK MAX with CAN ID {m.drive_can}")
            print(f"    the motor that steers the wheel = SPARK MAX with CAN ID {m.turn_can}")
            if S.STEERING_ENCODERS == "rio":
                print(f"    its steering encoder is plugged into roboRIO DIO port {m.encoder_dio}")
            print(f"    drive motor mounted {'under' if m.drive_motor_bottom else 'on top of'} the module, "
                  f"steering motor {'under' if m.turn_motor_bottom else 'on top of'} it")
            cans.setdefault(m.drive_can, []).append(f"{words} drive")
            cans.setdefault(m.turn_can, []).append(f"{words} steering")
            dios.setdefault(m.encoder_dio, []).append(words)
            if (m.drive_can, m.turn_can) == EXAMPLE_CAN_IDS[name]:
                notes.append(f"{name}: CAN IDs {m.drive_can} and {m.turn_can} are the example numbers. "
                             "Check them in REV Hardware Client (see the README); if they really are, ignore this.")
            for can in (m.drive_can, m.turn_can):
                if not 1 <= can <= 62:
                    notes.append(f"{name}: CAN ID {can} can't be right: SPARK MAX CAN IDs are 1 to 62.")
            if S.STEERING_ENCODERS == "rio" and not 0 <= m.encoder_dio <= 9:
                notes.append(f"{name}: DIO port {m.encoder_dio} can't be right: the roboRIO's DIO ports are 0 to 9.")

        for can, users in cans.items():
            if len(users) > 1:
                notes.append(f"CAN ID {can} is used twice ({' and '.join(users)}). Every SPARK MAX needs its own.")
        if S.STEERING_ENCODERS == "rio":
            for dio, users in dios.items():
                if len(users) > 1:
                    notes.append(f"DIO port {dio} is used twice ({' and '.join(users)}). Each encoder needs its own.")

        # ---- encoders
        where = "roboRIO DIO ports" if S.STEERING_ENCODERS == "rio" else "steering SPARK MAX data ports"
        print(f"Steering encoders: Through Bore Encoder version {S.ENCODER_VERSION}, plugged into the {where}")
        if str(S.STEERING_ENCODERS).lower() not in ("rio", "spark"):
            notes.append('STEERING_ENCODERS must be "rio" or "spark" (with the quote marks).')
        if S.ENCODER_VERSION not in (1, 2):
            notes.append("ENCODER_VERSION must be 1 or 2.")
        if S.STEERING_ZEROS_DEG is None:
            print("Steering zeros backup: none yet (setup step 4 makes one)")

        # ---- size
        print(f"Robot size: wheels {S.TRACK_WIDTH_M} m apart side to side, {S.WHEEL_BASE_M} m front to back")
        print(f"Wheels: {S.WHEEL_DIAMETER_M} m across")
        for label, value in (("TRACK_WIDTH_M", S.TRACK_WIDTH_M), ("WHEEL_BASE_M", S.WHEEL_BASE_M)):
            if value > 5:
                notes.append(f"{label} = {value} looks like centimeters. It's in meters: 60 cm is 0.60.")
            elif not 0.2 <= value <= 1.2:
                notes.append(f"{label} = {value} m is not a robot size. Measure between the wheel centers.")
        if S.TRACK_WIDTH_M == EXAMPLE_SIZE_M and S.WHEEL_BASE_M == EXAMPLE_SIZE_M:
            notes.append("TRACK_WIDTH_M and WHEEL_BASE_M are still the example numbers. Measure your robot "
                         "(if it really is 0.6858 m, ignore this).")
        if S.WHEEL_DIAMETER_M > 1:
            notes.append(f"WHEEL_DIAMETER_M = {S.WHEEL_DIAMETER_M} looks like centimeters or inches. It's in meters.")

        # ---- driving
        print(f"Driving: top speed {S.MAX_SPEED_MPS} m/s, top spin {S.MAX_TURN_DEG_PER_S} degrees per second, "
              f"controller in Driver Station USB slot {S.CONTROLLER_PORT}")
        if S.MAX_SPEED_MPS > 2.0:
            notes.append(f"MAX_SPEED_MPS = {S.MAX_SPEED_MPS}. For your first drives, 1.5 is a lot safer. "
                         "Raise it once everyone is comfortable. (Not a mistake, just a suggestion.)")

        # ---- gyro and cameras
        gyro = str(S.GYRO).lower()
        print(f'Gyro: navX2 plugged into the roboRIO\'s {"MXP port (on top)" if gyro == "mxp" else "USB port"}')
        if gyro not in ("mxp", "usb"):
            notes.append('GYRO must be "mxp" or "usb" (with the quote marks).')
        print(f"Cameras: {'ON' if S.VISION_ENABLED else 'off'} ({len(S.CAMERAS)} listed)")

        # ---- what to check
        if notes:
            print(f"==== {len(notes)} thing(s) to check in robot_settings.py: ====")
            for i, note in enumerate(notes, 1):
                print(f"  {i}. {note}")
        else:
            print("==== Nothing looks wrong. Next: examples/00_01_checkup ====")

    def autonomousInit(self) -> None:
        print("Nothing moves in this step: leave the robot disabled.")

    def teleopInit(self) -> None:
        print("Nothing moves in this step: leave the robot disabled.")
