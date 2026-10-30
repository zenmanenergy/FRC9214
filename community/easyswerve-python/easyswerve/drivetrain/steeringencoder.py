"""
Steering angle sensing for one EasySwerve module.

Two wirings are supported (ModuleConstants.kSteeringEncoderLocation):

  "rio"   The Through Bore Encoder's absolute output goes to a roboRIO DIO
          port (signal, 5 V, ground). The roboRIO can't feed that to the
          SPARK, so the SPARK steers with the NEO's built-in sensor (through
          the 20 : 1 steering gearing). The absolute encoder sets that sensor
          to the true angle at startup and corrects any drift (backlash,
          skipped counts) whenever the wheel is still, so the SPARK still runs
          its fast 1 kHz steering loop against the true angle.

  "spark" The encoder plugs into the turning SPARK MAX data port and the SPARK
          steers on it directly (REV's original setup).

Both give the module the same interface:
  getPosition()   steering angle, radians 0..2pi, 0 = the hex-key zero
  getVelocity()   steering speed, rad/s
"""

import math
import typing

import wpilib

from .constants import ModuleConstants

TAU = 2 * math.pi

# Pulse timing of the absolute output, per encoder version:
# (shortest pulse / period, longest pulse / period, frequency Hz)
_PULSE = {
    1: (1.0 / 1025.0, 1024.0 / 1025.0, 975.6),
    2: (3.884 / 1000.0, 998.06 / 1000.0, 1000.0),
}


def _wrapPi(a: float) -> float:
    return (a + math.pi) % TAU - math.pi


class SparkSteeringEncoder:
    """Through Bore Encoder on the turning SPARK MAX data port."""

    location = "spark"

    def __init__(self, turningSpark) -> None:
        self.encoder = turningSpark.getAbsoluteEncoder()

    def getPosition(self) -> float:
        return self.encoder.getPosition()

    def getVelocity(self) -> float:
        return self.encoder.getVelocity()

    def isConnected(self) -> bool:
        return True

    def sync(self, enabled: bool, now: float) -> None:
        pass  # the SPARK reads the absolute angle itself

    @property
    def synced(self) -> bool:
        return True


class RioSteeringEncoder:
    """Through Bore Encoder on a roboRIO DIO port, with the SPARK steering on
    the NEO's built-in sensor kept in sync with it."""

    location = "rio"

    def __init__(self, turningSpark, dioChannel: int, offset: float, name: str = "") -> None:
        self.name = name
        self.dioChannel = dioChannel
        self.offset = offset
        self.relative = turningSpark.getEncoder()  # radians, via conversion factor

        # get() returns 0..2pi
        self.absolute = wpilib.DutyCycleEncoder(dioChannel, TAU, 0.0)
        version = ModuleConstants.kThroughBoreEncoderVersion
        if version not in _PULSE:
            raise ValueError(
                f"ModuleConstants.kThroughBoreEncoderVersion must be 1 or 2, got {version!r}"
            )
        low, high, freq = _PULSE[version]
        self.absolute.setDutyCycleRange(low, high)
        self.absolute.setAssumedFrequency(freq)

        self._synced = False
        self._lastSync = -1e9
        self._lastWarn = -1e9
        self.resyncThreshold = math.radians(ModuleConstants.kSteeringResyncThresholdDeg)
        self.resyncCount = 0

        # Stuck-encoder check: if the wheel turns a lot while the absolute
        # reading doesn't change at all, stop trusting the absolute encoder.
        self.stuck = False
        self._lastRaw: typing.Optional[float] = None
        self._movedSinceRawChange = 0.0
        self._lastRelative: typing.Optional[float] = None

    # ---- the module-facing interface -----------------------------------------

    def getPosition(self) -> float:
        return self.relative.getPosition() % TAU

    def getVelocity(self) -> float:
        return self.relative.getVelocity()

    def isConnected(self) -> bool:
        return self.absolute.isConnected()

    @property
    def synced(self) -> bool:
        """True once the NEO sensor has been set from the absolute encoder."""
        return self._synced

    # ---- absolute angle ---------------------------------------------------------

    def rawAbsolute(self) -> typing.Optional[float]:
        """Absolute encoder reading with no offset applied (radians), or None if
        the encoder isn't sending a signal."""
        if not self.absolute.isConnected():
            return None
        return self.absolute.get() % TAU

    def absoluteAngle(self) -> typing.Optional[float]:
        raw = self.rawAbsolute()
        if raw is None:
            return None
        return (raw - self.offset) % TAU

    def setOffset(self, offset: float) -> None:
        self.offset = offset % TAU
        self._synced = False

    def sync(self, enabled: bool, now: float) -> None:
        """Called every loop. Sets the NEO sensor to the absolute angle:
        always while disabled (twice a second), and while enabled only when the
        wheel is still and the two disagree by more than the threshold."""
        actual = self.absoluteAngle()
        if actual is None:
            if now - self._lastWarn > 5.0:
                self._lastWarn = now
                wpilib.reportWarning(
                    f"Steering encoder {self.name} on DIO {self.dioChannel} has no signal; "
                    "check its 3 wires (signal, 5 V, ground)"
                )
            return

        current = self.relative.getPosition()
        self._checkStuck(current, now)
        if self.stuck:
            return
        diff = _wrapPi(actual - current % TAU)

        if not self._synced:
            self._setRelative(current + diff)
            self._synced = True
            self._lastSync = now
            return

        if not enabled:
            due = now - self._lastSync >= 0.5 and abs(diff) > math.radians(0.1)
        else:
            still = abs(self.relative.getVelocity()) < 0.05
            due = still and abs(diff) > self.resyncThreshold and now - self._lastSync >= 1.0
        if due:
            self._setRelative(current + diff)
            self._lastSync = now
            self.resyncCount += 1

    STUCK_TURN_RAD = math.radians(45.0)

    def _setRelative(self, value: float) -> None:
        self.relative.setPosition(value)
        self._lastRelative = value  # a correction is not wheel movement

    def _checkStuck(self, current: float, now: float) -> None:
        raw = self.rawAbsolute()
        if self._lastRelative is not None:
            self._movedSinceRawChange += abs(current - self._lastRelative)
        self._lastRelative = current
        if raw is None:
            return
        if self._lastRaw is None or abs(_wrapPi(raw - self._lastRaw)) > 1e-6:
            self._lastRaw = raw
            self._movedSinceRawChange = 0.0
            if self.stuck:
                self.stuck = False
                wpilib.reportWarning(
                    f"Steering encoder {self.name} on DIO {self.dioChannel} is changing again; trusting it"
                )
            return
        if not self.stuck and self._movedSinceRawChange > self.STUCK_TURN_RAD:
            self.stuck = True
            wpilib.reportWarning(
                f"Steering encoder {self.name} on DIO {self.dioChannel} reads the same value while "
                f"the wheel turned {math.degrees(self._movedSinceRawChange):.0f} deg; ignoring it "
                "(check the magnet and wiring)"
            )
