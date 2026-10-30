"""
VisionSubsystem: feeds PhotonVision AprilTag position estimates into the
EasySwerve drivetrain's pose estimator.

How it degrades (it never stops the robot from driving):

  photonlibpy not installed   -> wheels + gyro only, one warning at startup
  VISION_ENABLED = False      -> wheels + gyro only
  Orange Pi off / unplugged   -> wheels + gyro only; picks cameras up as soon
                                 as they appear, no restart needed
  camera not measured yet     -> "watch only": dashboard shows what it sees,
                                 but it doesn't move the robot's position
  bad frame (ambiguous, far,  -> that frame ignored, reason counted on the
  spinning, off the field)       dashboard
  any unexpected error        -> that camera skipped for the loop, error
                                 reported (rate limited)

Create it after the DriveSubsystem, and name the attribute "vision" so the
simulator finds it:

    self.vision = VisionSubsystem(self.robotDrive)
"""

import collections
import math
import typing

import commands2
import wpilib
from wpimath.geometry import Pose2d, Rotation3d, Transform3d, Translation3d

from . import config as C
from . import filters

try:
    from photonlibpy import PhotonCamera, PhotonPoseEstimator
    from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

    _IMPORT_ERROR: typing.Optional[Exception] = None
except Exception as e:  # not installed, or a version problem
    _IMPORT_ERROR = e


def robotToCamera(cam: C.CameraConfig) -> Transform3d:
    """Camera position and direction on the robot, as PhotonVision expects.
    (WPILib's pitch is positive nose-down, hence the minus on tiltUpDeg.)"""
    return Transform3d(
        Translation3d(cam.forwardM, cam.leftM, cam.upM),
        Rotation3d(
            math.radians(cam.rollDeg),
            math.radians(-cam.tiltUpDeg),
            math.radians(cam.yawDeg),
        ),
    )


class _Camera:
    def __init__(self, cfg: C.CameraConfig, layout) -> None:
        self.cfg = cfg
        self.camera = PhotonCamera(cfg.name)
        self.estimator = PhotonPoseEstimator(layout, robotToCamera(cfg))
        self.usable, self.usableReason = filters.cameraUsable(
            cfg, wpilib.RobotBase.isSimulation()
        )
        self.connected = False
        self.accepted = 0
        self.watched = 0
        self.rejected: typing.Dict[str, int] = collections.Counter()
        self.lastReason = ""
        self.lastTags = 0
        self.lastPose: typing.Optional[Pose2d] = None
        self.lastSeen = -1e9
        self.errors = 0
        self.lastErrorReport = -1e9


class VisionSubsystem(commands2.Subsystem):
    def __init__(self, drive) -> None:
        """:param drive: the EasySwerve DriveSubsystem"""
        super().__init__()
        self.drive = drive
        self.cameras: typing.List[_Camera] = []
        self.layout = None
        self.status = "off"
        self._sim = None
        self._simFailed = False
        self._lastSeed = -1e9
        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Vision/Field", self.field)

        if not C.VISION_ENABLED:
            self.status = "off (VISION_ENABLED = False): wheels + gyro only"
        elif _IMPORT_ERROR is not None:
            self.status = "photonlibpy not installed: wheels + gyro only"
            wpilib.reportWarning(
                f"Vision disabled: could not import photonlibpy ({_IMPORT_ERROR}). "
                "Add photonlibpy to requires in pyproject.toml."
            )
        else:
            try:
                self.layout = self._loadLayout()
                self.cameras = [_Camera(c, self.layout) for c in C.CAMERAS if c.enabled]
                self.status = "starting"
            except Exception as e:
                self.cameras = []
                self.status = f"setup failed ({e}): wheels + gyro only"
                wpilib.reportError(f"Vision setup failed: {e}", False)

        print(f"[vision] {self.status}")
        for cam in self.cameras:
            print(f"[vision] camera '{cam.cfg.name}': {cam.usableReason}")

    @staticmethod
    def _loadLayout():
        if C.FIELD_LAYOUT:
            return AprilTagFieldLayout.loadField(getattr(AprilTagField, C.FIELD_LAYOUT))
        return AprilTagFieldLayout.loadField(AprilTagField.kDefaultField)

    # ---- every loop ----------------------------------------------------------------

    def periodic(self) -> None:
        now = wpilib.Timer.getFPGATimestamp()
        for cam in self.cameras:
            try:
                self._processCamera(cam, now)
            except Exception as e:  # never let vision break the robot loop
                cam.errors += 1
                if now - cam.lastErrorReport > 5.0:
                    cam.lastErrorReport = now
                    wpilib.reportError(f"Vision camera '{cam.cfg.name}': {e}", False)
        self._updateStatus(now)
        self._publish()

    def _processCamera(self, cam: _Camera, now: float) -> None:
        cam.connected = cam.camera.isConnected()
        # Always drain the queue, even when not fusing, so stale frames never pile up
        for result in cam.camera.getAllUnreadResults():
            multi = cam.estimator.estimateCoprocMultiTagPose(result)
            est = multi if multi is not None else cam.estimator.estimateLowestAmbiguityPose(result)
            if est is None:
                continue
            cam.lastSeen = now

            tags = [t for t in est.targetsUsed if self.layout.getTagPose(t.fiducialId) is not None]
            if multi is not None:
                used = tags
                ambiguity = 0.0
            else:
                good = [t for t in tags if t.poseAmbiguity >= 0]
                if not good:
                    continue
                best = min(good, key=lambda t: t.poseAmbiguity)
                used = [best]
                ambiguity = best.poseAmbiguity
            if not used:
                continue

            avgDistance = sum(
                t.getBestCameraToTarget().translation().norm() for t in used
            ) / len(used)
            pose3 = est.estimatedPose
            verdict = filters.judge(
                filters.Estimate(
                    x=pose3.X(),
                    y=pose3.Y(),
                    z=pose3.Z(),
                    tagCount=len(used),
                    avgDistanceM=avgDistance,
                    ambiguity=ambiguity,
                    spinDegPerS=self.drive.getTurnRate(),
                    fieldLengthM=self.layout.getFieldLength(),
                    fieldWidthM=self.layout.getFieldWidth(),
                )
            )
            pose2 = pose3.toPose2d()
            cam.lastPose = pose2
            cam.lastTags = len(used)
            if not verdict.accepted:
                cam.rejected[verdict.reason] += 1
                cam.lastReason = verdict.reason
                continue
            if not cam.usable:
                cam.watched += 1
                continue

            if (
                wpilib.DriverStation.isDisabled()
                and C.SEED_WHILE_DISABLED_MIN_TAGS > 0
                and len(used) >= C.SEED_WHILE_DISABLED_MIN_TAGS
            ):
                # Before a match: put the robot exactly where the cameras say
                if now - self._lastSeed > 0.5:
                    self._lastSeed = now
                    self.drive.resetOdometry(pose2)
                cam.accepted += 1
                continue

            self.drive.addVisionMeasurement(pose2, est.timestampSeconds, verdict.stdDevs)
            cam.accepted += 1

    def _updateStatus(self, now: float) -> None:
        if not self.cameras:
            return
        fusing = [c for c in self.cameras if c.connected and c.usable]
        if fusing:
            names = ", ".join(c.cfg.name for c in fusing)
            self.status = f"wheels + gyro + camera ({names})"
        elif any(c.connected for c in self.cameras):
            self.status = "camera connected but watch only (not measured): wheels + gyro"
        else:
            self.status = "no camera connected: wheels + gyro only"

    def _publish(self) -> None:
        sd = wpilib.SmartDashboard
        sd.putString("Vision/Status", self.status)
        sd.putNumber("Vision/Measurements fused", getattr(self.drive, "visionMeasurementCount", 0))
        self.field.setRobotPose(self.drive.getPose())
        for cam in self.cameras:
            k = f"Vision/{cam.cfg.name}/"
            sd.putBoolean(k + "connected", cam.connected)
            sd.putString(k + "mode", cam.usableReason)
            sd.putNumber(k + "tags seen", cam.lastTags)
            sd.putNumber(k + "accepted", cam.accepted)
            sd.putNumber(k + "watch only", cam.watched)
            sd.putNumber(k + "rejected", sum(cam.rejected.values()))
            sd.putString(k + "last reject reason", cam.lastReason)
            sd.putNumber(k + "errors", cam.errors)
            if cam.lastPose is not None:
                self.field.getObject(cam.cfg.name).setPose(cam.lastPose)

    # ---- simulator ------------------------------------------------------------------

    def simulationUpdate(self, truePose: Pose2d) -> None:
        """Called by physics.py every loop with the robot's true position, so
        simulated cameras see the simulated AprilTags. Not used on the robot."""
        if not self.cameras or self._simFailed or not wpilib.RobotBase.isSimulation():
            return
        try:
            if self._sim is None:
                self._sim = self._makeSim()
            self._sim.update(truePose)
        except Exception as e:
            self._simFailed = True
            print(f"[vision] camera simulation off ({e}); robot code unaffected")

    def _makeSim(self):
        from photonlibpy.simulation import PhotonCameraSim, SimCameraProperties, VisionSystemSim

        sim = VisionSystemSim("main")
        sim.addAprilTags(self.layout)
        for cam in self.cameras:
            props = getattr(SimCameraProperties, C.SIM_CAMERA_MODEL)()
            props.setFPS(C.SIM_CAMERA_FPS)
            props.setAvgLatency(C.SIM_CAMERA_LATENCY_MS)
            camSim = PhotonCameraSim(cam.camera, props, self.layout)
            sim.addCamera(camSim, robotToCamera(cam.cfg))
        print(f"[vision] simulating {len(self.cameras)} camera(s), model {C.SIM_CAMERA_MODEL}")
        return sim
