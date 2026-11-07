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
and read the console (it prints once, 2 seconds after the code starts). Fix
anything it lists, deploy again, and repeat.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy

The comments on the right explain each line. Python ignores everything
after a # sign, so they don't change what the code does.
"""

import wpilib                                                        # WPILib: the library all FRC robots use
import robot_settings as S                                           # load robot_settings.py: S.GYRO is its GYRO

EXAMPLE_CAN_IDS = {                                                  # the example CAN IDs, per module:
    "FRONT_LEFT": (9, 8),                                            # (drive motor, steering motor)
    "FRONT_RIGHT": (11, 10),                                         # if yours still match, you probably
    "BACK_LEFT": (15, 14),                                           # haven't checked them yet
    "BACK_RIGHT": (13, 12),
}                                                                    # } ends the dictionary (name: value pairs)
EXAMPLE_SIZE_M = 0.6858                                              # the example robot size (meters)

MODULES = [                                                          # the four corners: (setting, words to print)
    ("FRONT_LEFT", "front left"),
    ("FRONT_RIGHT", "front right"),
    ("BACK_LEFT", "back left"),
    ("BACK_RIGHT", "back right"),
]                                                                    # ] ends the list


class MyRobot(wpilib.TimedRobot):                                    # our robot: runs 50 times a second
    def robotInit(self):                                             # runs ONCE, when the robot code starts
        self.reported = False                                        # printed the report yet?
        self.startTime = wpilib.Timer.getFPGATimestamp()             # the time now (seconds)

    def robotPeriodic(self):                                         # runs 50 times a second, in every mode
        if self.reported:                                            # already printed it?
            return                                                   # then do nothing: it prints only once
        now = wpilib.Timer.getFPGATimestamp()                        # the time now, in seconds
        if now - self.startTime < 2.0:                               # in the first 2 seconds after starting...
            return                                                   # ...wait (the console is connecting)
        self.reported = True                                         # remember that we've printed it
        self.report()                                                # print the report (the function below)

    def report(self):                                                # print settings and things to check
        notes = []                                                   # empty list: problems get added to it
        cans = {}                                                    # CAN ID -> motors using it
        dios = {}                                                    # DIO port -> encoders using it

        print("==== Your code is on the robot! ====")                # proof that the deploy worked
        print("This is what robot_settings.py says:")

        for name, words in MODULES:                                  # repeat this block for each of the 4 modules
            m = getattr(S, name)                                     # that module's settings, e.g. S.FRONT_LEFT
            print(words.upper() + " module:")                        # e.g. "FRONT LEFT module:"
            print("  drive motor CAN ID:", m.drive_can)              # SPARK MAX that spins the wheel
            print("  steering motor CAN ID:", m.turn_can)            # SPARK MAX that turns the wheel
            print("  encoder DIO port:", m.encoder_dio)              # where its steering encoder plugs in
            print("  drive motor under:", m.drive_motor_bottom)      # under the plate? True or False
            print("  steer motor under:", m.turn_motor_bottom)       # under the plate? True or False

            drive = words + " drive"                                 # e.g. "front left drive"
            steer = words + " steering"                              # and for its steering motor
            cans.setdefault(m.drive_can, []).append(drive)           # note who uses this CAN ID
            cans.setdefault(m.turn_can, []).append(steer)            # same for the steering motor
            dios.setdefault(m.encoder_dio, []).append(words)         # note who uses this DIO port

            ids = (m.drive_can, m.turn_can)                          # this module's two CAN IDs
            if ids == EXAMPLE_CAN_IDS[name]:                         # still the example numbers?
                msg = ": example CAN IDs. Check them!"               # (REV Hardware Client: see README)
                notes.append(name + msg)                             # add it to the list of notes
            for can in ids:                                          # check each of the two CAN IDs
                if can < 1 or can > 62:                              # SPARK MAX CAN IDs must be 1 to 62
                    msg = ": CAN IDs must be 1 to 62."
                    notes.append(name + msg)
            dio = m.encoder_dio                                      # this module's DIO port
            if dio < 0 or dio > 9:                                   # the roboRIO's DIO ports are 0 to 9
                notes.append(name + ": DIO must be 0 to 9.")

        for can, users in cans.items():                              # every CAN ID used, and who uses it
            if len(users) > 1:                                       # more than one motor on the same ID?
                who = " and ".join(users)                            # e.g. "front left drive and back left drive"
                msg = "CAN ID " + str(can) + " used twice: "         # str() turns a number into text
                notes.append(msg + who)                              # every SPARK MAX needs its own ID
        for dio, users in dios.items():                              # every DIO port used, and who uses it
            if len(users) > 1:                                       # more than one encoder on the same port?
                who = " and ".join(users)
                msg = "DIO port " + str(dio) + " used twice: "
                notes.append(msg + who)                              # every encoder needs its own port

        where = str(S.STEERING_ENCODERS).lower()                     # "rio" or "spark", in small letters
        print("Steering encoders plugged into:", where)              # the roboRIO or the SPARK MAX
        if where not in ("rio", "spark"):                            # anything else is a typo
            notes.append('STEERING_ENCODERS: "rio" or "spark"')
        version = S.ENCODER_VERSION                                  # Through Bore Encoder version
        print("Through Bore Encoder version:", version)              # 1 or 2
        if version not in (1, 2):                                    # anything else is a typo
            notes.append("ENCODER_VERSION must be 1 or 2.")

        track = S.TRACK_WIDTH_M                                      # left wheels to right wheels (m)
        base = S.WHEEL_BASE_M                                        # front wheels to back wheels (m)
        wheel = S.WHEEL_DIAMETER_M                                   # wheel diameter (m)
        print("Side to side:", track, "m")
        print("Front to back:", base, "m")
        print("Wheel diameter:", wheel, "m")
        for label, value in (("TRACK_WIDTH_M", track),               # check both robot sizes,
                             ("WHEEL_BASE_M", base)):                # one at a time
            if value > 5:                                            # no robot is 5 m wide, so...
                notes.append(label + ": in cm? 60 cm = 0.60")        # ...it was typed in cm
            elif value < 0.2 or value > 1.2:                         # should be 0.2 to 1.2 m
                notes.append(label + ": not a robot size.")          # measure wheel center to center
        if track == EXAMPLE_SIZE_M and base == EXAMPLE_SIZE_M:       # both unchanged?
            notes.append("Robot size: example numbers. Measure!")
        if wheel > 1:                                                # a wheel over 1 m across can't be right
            notes.append("WHEEL_DIAMETER_M is in meters.")           # (0.1016 for new wheels)

        length = S.CARPET_LENGTH_M                                   # practice carpet, long side (m)
        width = S.CARPET_WIDTH_M                                     # its short side (m)
        robotL = S.ROBOT_LENGTH_M                                    # robot with bumpers, front to back (m)
        robotW = S.ROBOT_WIDTH_M                                     # and left to right (m)
        print("Robot with bumpers:", robotL, "m by", robotW, "m")
        if robotL > 5 or robotW > 5:                                 # no robot is 5 m long, so...
            notes.append("ROBOT_LENGTH_M: in cm? 95 cm = 0.95")      # ...it was typed in cm
        elif robotL < base or robotW < track:                        # smaller than its own wheels?
            notes.append("ROBOT_LENGTH_M: measure outside the bumpers")
        if (robotL, robotW) == (0.95, 0.95):                         # still the example size?
            notes.append("ROBOT size: example numbers. Measure!")
        if length is None:                                           # None = no carpet (a big floor)
            print("Practice carpet: none (moves are not limited)")
        else:
            print("Practice carpet:", length, "m by", width, "m")
            if length > 30 or width > 30:                            # no carpet is 30 m, so...
                notes.append("CARPET: in cm? 274 cm = 2.74")         # ...it was typed in cm
            elif width < robotW + 0.2 or length < robotL + 0.4:      # hardly bigger than the robot?
                notes.append("CARPET: too small for the robot")
            if (length, width) == (2.74, 1.52):                      # still the example size?
                notes.append("CARPET: example size. Measure it!")

        speed = S.MAX_SPEED_MPS                                      # top speed, meters per second
        spin = S.MAX_TURN_DEG_PER_S                                  # top spin, degrees per second
        port = S.CONTROLLER_PORT                                     # Driver Station USB slot
        print("Top speed:", speed, "m/s")
        print("Top spin:", spin, "degrees per second")
        print("Controller in USB slot", port)
        if speed > 2.0:                                              # fast for a first drive
            notes.append("MAX_SPEED_MPS: try 1.5 at first.")         # (only a suggestion)

        gyro = str(S.GYRO).lower()                                   # "mxp" or "usb", in small letters
        print("Gyro (navX2) plugged into:", gyro)                    # which roboRIO port
        if gyro not in ("mxp", "usb"):                               # anything else is a typo
            notes.append('GYRO must be "mxp" or "usb".')
        print("Heading hold:", S.HEADING_HOLD)                       # gyro keeps it straight (True = on)
        print("Cameras on:", S.VISION_ENABLED)                       # True or False (docs/VISION.md)

        if len(notes) == 0:                                          # no notes: nothing looked wrong
            print("==== Nothing looks wrong. ====")
            print("Next: examples/00_01_checkup")
        else:                                                        # otherwise, print the notes
            count = str(len(notes))                                  # how many notes, as text
            print("==== " + count + " thing(s) to check ====")       # in robot_settings.py
            number = 1                                               # start counting at 1
            for note in notes:                                       # repeat for each note in the list
                print("  " + str(number) + ". " + note)              # e.g. "  1. GYRO must be..."
                number = number + 1                                  # add 1 for the next note

    def autonomousInit(self):                                        # runs once when enabled in Autonomous
        print("Nothing moves here: stay disabled.")

    def teleopInit(self):                                            # runs once when enabled in Teleoperated
        print("Nothing moves here: stay disabled.")
