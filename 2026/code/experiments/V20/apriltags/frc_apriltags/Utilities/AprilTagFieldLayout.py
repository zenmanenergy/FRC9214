# Import Libraries
import json
import numpy as np
from   enum import Enum
from   typing import Sequence
from   importlib import resources
from   wpimath.geometry import *

# Import Utilities
from frc_apriltags.Utilities import Logger, AprilTag

# Creates the AprilTagFieldLayout class
class AprilTagFieldLayout:
    """
    A class to store the positions of all known AprilTags

    :param tags: A list of all known tags.
    :param fieldLength: The length (y) of the field, in meters.
    :param fieldWidth: The width (x) of the field, in meters.
    :param isRed: If you are on the red alliance.
    """
    class Origin(Enum):
        kBlueAllianceWallRightSide = "BlueWall"
        kRedAllianceWallRightSide  = "RedWall"

    def __init__(self, tags: Sequence[AprilTag], fieldLength: float, fieldWidth: float, isRed: bool = False):
        """
        Generates an field with AprilTags for testing.

        :param tags: A list of all known tags.
        :param fieldLength: The length (y) of the field, in meters.
        :param fieldWidth: The width (x) of the field, in meters.
        ;param isRed: If you are on the red alliance.
        """
        # Localize parameters
        self.fieldLength = fieldLength
        self.fieldWidth = fieldWidth

        # Logs the field size
        Logger.logInfo(f"Field length: {self.fieldLength}, Field width: {self.fieldWidth}", True)

        # Creates the allTags array
        self.allTags = [Pose3d()] * 9

        # Sorts the data from tags into allTags
        for tag in tags:
            id   = tag.getId()
            pose = tag.getPose()

            self.allTags[id] = pose

            # Logs the tag information
            Logger.logInfo(f"Tag {id}. Pose: {pose}", True)

        # Variables
        self.m_origin  = None
        self.logStatus = False

        # Sets the origin depending on alliance
        if (isRed == True):
            self.setOrigin(AprilTagFieldLayout.Origin.kRedAllianceWallRightSide)
        elif (isRed == False):
            self.setOrigin(AprilTagFieldLayout.Origin.kBlueAllianceWallRightSide)

    @classmethod
    def fromJson(cls, name: str, isRed: bool):
        """
        Generates an field with AprilTags from the WPILib json file.

        :param name: The name of the json file in the year-gamename format.
        :param isRed: If you are on the red alliance.
        """
        # Creates a blank array
        allTags = [AprilTag()] * 9

        # Returns the JSON as a dictionary
        with resources.open_binary("frc_apriltags.Tag_Layouts", name + ".json") as fp:
            data = json.load(fp)

        # Loops through the json data
        for tag in data["tags"]:
            # Gets the tag ID
            id = tag["ID"]

            # Gets the tag's translation data
            x_trans = tag["pose"]["translation"]["x"]
            y_trans = tag["pose"]["translation"]["y"]
            z_trans = tag["pose"]["translation"]["z"]

            # Gets the tag's rotation data
            w_rot = tag["pose"]["rotation"]["quaternion"]["W"]
            x_rot = tag["pose"]["rotation"]["quaternion"]["X"]
            y_rot = tag["pose"]["rotation"]["quaternion"]["Y"]
            z_rot = tag["pose"]["rotation"]["quaternion"]["Z"]

            # Creates a quaternion
            q = Quaternion(w_rot, x_rot, y_rot, z_rot)

            # Creates a Pose3d object
            tag = AprilTag.fromQuaternion(id, x_trans, y_trans, z_trans, q)

            # Adds the tag to the allTags array
            allTags[id] = tag

        # Sets the field dimensions
        fieldLength = data["field"]["length"]
        fieldWidth  = data["field"]["width"]

        # Return the class
        return cls(allTags, fieldLength, fieldWidth, isRed)

    def setOrigin(self, origin):
        """
        Sets the origin depending on the alliance color. The origins are calculated from the field dimensions.
        This transforms the ``Pose3d``'s returned by ``getTagPose()`` to return the correct pose relative to a predefined coordinate frame.

        :param origin: The predefined origin.
        """
        # Sets the origin
        if (origin == AprilTagFieldLayout.Origin.kBlueAllianceWallRightSide):
            self.m_origin = Pose3d()
        elif (origin == AprilTagFieldLayout.Origin.kRedAllianceWallRightSide):
            self.m_origin = Pose3d(
                Translation3d(self.fieldLength, self.fieldWidth, 0),
                Rotation3d(0, 0, np.pi)
            )
        else:
            origin = None
            raise ValueError("Unsupported enumerator value.")
        
        # Logs the origin
        Logger.logInfo(f"Origin at {origin}", self.logStatus)

    def getTags(self):
        """
        Returns all the created tags

        :return: All the tags on the field.
        """
        return self.allTags

    def getTagPose(self, id: int) -> Pose3d:
        """
        Returns the ``Pose3d`` of the selected tag.

        :return: A WPILib ``Pose3d`` object.
        """
        if (type(self.allTags[id]) is not Pose3d):
            return Pose3d()
        else:
            return self.allTags[id].relativeTo(self.m_origin)

    def enableLogging(self):
        """
        Enables logging for this class.
        """
        self.logStatus = True