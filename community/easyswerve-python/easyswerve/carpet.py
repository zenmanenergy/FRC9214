"""
The practice carpet: how much room the robot has, worked out from
CARPET_LENGTH_M, CARPET_WIDTH_M, ROBOT_LENGTH_M and ROBOT_WIDTH_M in
robot_settings.py. The examples use it so their moves stay on the carpet.

Two places to start the robot, always facing the long way:

  In the middle (the examples 01 to 36): the robot's center on the carpet's
  center. It can go FORWARD_M forward or back, and SIDE_M left or right.

  At one end (setup steps 8, 10 and 11, which need a longer straight run):
  back bumper about 20 cm from the edge, centered side to side. It can go
  END_AHEAD_M forward.

Each room leaves a 10 cm margin. CARPET_LENGTH_M = None means no carpet (a
big floor or a real field): then nothing is limited.

    from easyswerve import carpet
    meters = carpet.fitMeters(2.0)                    # 2 m, or less to fit
    seconds = carpet.fitSeconds(2.0, forward=0.25)    # 2 s, or less to fit
"""

import math

import robot_settings as RS

MARGIN_M = 0.10  # never closer than this to an edge
END_GAP_M = 0.20  # at one end: back bumper this far from the edge (room to turn)

LENGTH_M = getattr(RS, "CARPET_LENGTH_M", None)
WIDTH_M = getattr(RS, "CARPET_WIDTH_M", None)
ROBOT_LENGTH_M = getattr(RS, "ROBOT_LENGTH_M", RS.WHEEL_BASE_M + 0.27)
ROBOT_WIDTH_M = getattr(RS, "ROBOT_WIDTH_M", RS.TRACK_WIDTH_M + 0.27)

ON = LENGTH_M is not None and WIDTH_M is not None
BIG = 1000.0  # "no limit"

if ON:
    FORWARD_M = max(0.0, (LENGTH_M - ROBOT_LENGTH_M) / 2 - MARGIN_M)
    SIDE_M = max(0.0, (WIDTH_M - ROBOT_WIDTH_M) / 2 - MARGIN_M)
    END_AHEAD_M = max(0.0, LENGTH_M - END_GAP_M - ROBOT_LENGTH_M - MARGIN_M)
    # turning on the spot: the corners swing out to half the diagonal
    CAN_SPIN = math.hypot(ROBOT_LENGTH_M, ROBOT_WIDTH_M) <= WIDTH_M
else:
    FORWARD_M = SIDE_M = END_AHEAD_M = BIG
    CAN_SPIN = True


def describe() -> str:
    """One line for the console."""
    if not ON:
        return "no carpet set (CARPET_LENGTH_M = None): moves are not limited"
    text = (
        f"carpet {LENGTH_M:.2f} m x {WIDTH_M:.2f} m, robot {ROBOT_LENGTH_M:.2f} m x "
        f"{ROBOT_WIDTH_M:.2f} m: from the middle {FORWARD_M:.2f} m forward or back and "
        f"{SIDE_M:.2f} m to each side; from one end {END_AHEAD_M:.2f} m forward"
    )
    if not CAN_SPIN:
        text += ". TOO NARROW to turn on the spot"
    return text


def fitMeters(meters: float, room: float = None) -> float:
    """meters, or less if the carpet is shorter (same sign: negative =
    backward). room: how much there is (FORWARD_M unless given)."""
    if room is None:
        room = FORWARD_M
    if abs(meters) > room:
        print(f"[carpet] {abs(meters):.2f} m shortened to {room:.2f} m to stay on the carpet")
    return math.copysign(min(abs(meters), room), meters)


def fitSeconds(seconds: float, forward: float = 0.0, left: float = 0.0, turn: float = 0.0) -> float:
    """How long driveFor can run with these speeds (fractions of full speed,
    as in driveFor) before the robot would leave the carpet, starting in the
    middle. Returns seconds, or less. It follows the motion 20 ms at a time,
    so arcs work too."""
    if not ON:
        return seconds
    from .drivetrain.constants import DriveConstants

    vx = forward * DriveConstants.kMaxSpeedMetersPerSecond
    vy = left * DriveConstants.kMaxSpeedMetersPerSecond
    w = turn * DriveConstants.kMaxAngularSpeed
    x = y = heading = t = 0.0
    while t < seconds:
        x += (vx * math.cos(heading) - vy * math.sin(heading)) * 0.02
        y += (vx * math.sin(heading) + vy * math.cos(heading)) * 0.02
        heading += w * 0.02
        if abs(x) > FORWARD_M or abs(y) > SIDE_M:
            print(f"[carpet] {seconds:.1f} s shortened to {t:.2f} s to stay on the carpet")
            return round(t, 2)
        t += 0.02
    return seconds
