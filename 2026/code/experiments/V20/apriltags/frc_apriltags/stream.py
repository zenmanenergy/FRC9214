# Import Libraries
import numpy as np
from   cscore import CameraServer as CS

# Import Utilities
from .Utilities import Logger

# Creates the BasicStreaming Class
class Streaming():
    """
    Use this class to stream unprocessed images to the driver station.

    :param camNum: The camera number.
    :param path: Can be found on Linux by running ``find /dev/v4l``.
    :param resolution: Width by height.
    :param fps: Frames per second.
    """
    def __init__(self, camNum: int, path: str = None, resolution: tuple = (640, 480), fps: int = 15) -> None:
        """
        Constructor for the BasicStreaming class.

        :param camNum: The camera number.
        :param path: Can be found on Linux by running ``find /dev/v4l``.
        :param resolution: Width by height.
        :param fps: Frames per second.
        """
        # Variables
        self.resolution = resolution

        # Creates a CameraServer
        CS.enableLogging()

        # Captures from a specified USB Camera on the system
        if (path is not None):
            # If path is known, use the path
            self.camera = CS.startAutomaticCapture(name = "Camera" + str(camNum), path = path)
        else:
            # Path is unknown, use the camera number
            self.camera = CS.startAutomaticCapture(dev = camNum)
        self.camera.setResolution(resolution[0], resolution[1])
        self.camera.setFPS(fps)

        # Gets images from the camera
        self.sink = CS.getVideo(self.camera)

        # Creates an output stream
        self.outputStream = CS.putVideo(name = "Processed Camera" + str(camNum), width = resolution[0], height = resolution[1])

        # Preallocates for images
        self.img = self.prealocateSpace()

        # Updates log
        Logger.logInfo("Stream initialized for camera" + str(camNum), True)
    
    def prealocateSpace(self):
        """
        Prealocates space for the stream.

        :return: An array of zeros with the same resolution as the camera.
        """
        return np.zeros(shape = (self.resolution[1], self.resolution[0], 3), dtype = np.uint8)

    def getStream(self):
        """
        Grabs a frame from the stream.

        :return: A frame.
        """
        time, self.img = self.sink.grabFrame(self.img)

        return self.img

    def streamImage(self, image):
        """
        Streams the camera back to ShuffleBoard for driver use.

        :param img: A processed stream.
        :return: The processed stream.
        """
        # Variables
        img = None

        # Logic to determine what to send
        if (image is not None):
            img = image
        else:
            img = self.getStream()

        # Sends a processed image back to ShuffleBoard
        self.outputStream.putFrame(img)

        return img

    def enableLogging(self):
        """
        Enables logging for this class.
        """
        self.logStatus = True