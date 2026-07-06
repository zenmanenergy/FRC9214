# Import Libraries
from wpimath.geometry import *

# Start of the AprilTag class
class AprilTag:
    """
    Use this class to generate an object with the properties of an AprilTag.

    :param id: The id of the AprilTag.
    :param pose: The ``Pose3d`` of the AprilTag.
    """
    def __init__(self, id: int = 0, pose: Pose3d = Pose3d()):
        """
        Constructor for the AprilTag class.

        :param id: The id of the AprilTag.
        :param pose: The ``Pose3d`` of the AprilTag.
        """
        # Localize parameters
        self.id = id
        self.pose = pose

    @classmethod
    def fromPoseComponents(cls, id: int, trans: Translation3d, rot: Rotation3d):
        """
        Constructor for the AprilTag class.

        :param id: The id of the AprilTag.
        :param trans: The ``Translation3d`` of the AprilTag.
        :param rot: The ``Rotation3d`` of the AprilTag.
        """
        # Generate 3D parts
        pose = Pose3d(trans, rot)

        # Return the class
        return AprilTag(id, pose)

    @classmethod
    def fromMatrix(cls, id: int, x: float, y: float, z: float, rMatrix):
        """
        Constructor for the AprilTag class.

        :param id: The id of the AprilTag.
        :param x: The x offset in meters.
        :param y: The y offset in meters.
        :param z: The z offset in meters.
        :param rMatrix: The 3x3 Rotation Matrix.
        """
        # Generate 3D parts
        rot   = Rotation3d(rMatrix)
        trans = Translation3d(x, y, z)
        pose   = Pose3d(trans, rot)

        # Return the class
        return AprilTag(id, pose)

    @classmethod
    def fromQuaternion(cls, id: int, x: float, y: float, z: float, q: Quaternion):
        """
        Constructor for the AprilTag class.

        :param id: The id of the AprilTag.
        :param x: The x offset in meters.
        :param y: The y offset in meters.
        :param z: The z offset in meters.
        :param q: The 4x4 Quaterion.
        """
        # Generate 3D parts
        rot   = Rotation3d(q)
        trans = Translation3d(x, y, z)
        pose  = Pose3d(trans, rot)

        # Return the class
        return AprilTag(id, pose)

    @classmethod
    def fromDetailed(cls, id: int, x: float, y: float, z: float, roll: float, pitch: float, yaw: float):
        """
        Constructor for the AprilTag class.

        :param id: The id of the AprilTag.
        :param x: The x offset in meters.
        :param y: The y offset in meters.
        :param z: The z offset in meters.
        :param roll: Rotaton around the tag's z axis in radians.
        :param pitch: Rotation around the tags's x axis in radians.
        :param yaw: Rotation around the tag's y axis in radians.
        """
        # Generate 3D parts
        rot   = Rotation3d(roll, pitch, yaw)
        trans = Translation3d(x, y, z)
        pose  = Pose3d(trans, rot)

        # Return the class
        return AprilTag(id, pose)

    def getId(self) -> int:
        """
        Gets the id of the given tag.

        :return: The id of the AprilTag.
        """
        return self.id

    def getPose(self) -> Pose3d:
        """
        Gets the pose of the given tag.

        :return: The ``Pose3d`` of the AprilTag.
        """
        return self.pose