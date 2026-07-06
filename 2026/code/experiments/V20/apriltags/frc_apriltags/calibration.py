# Import Libraries
import os
import glob
import cv2   as cv
import numpy as np

# Import Utilities
from .Utilities import Logger

# Defines the dimensions of the chessboard
CHESSBOARD = (8, 5)  # Number of interior corners (width in squares - 1 x height in squares - 1)

# Default termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Creates the Calibrate class
class Calibrate:
    """
    Use this class to calibrate your USBCamera.

    :param cap: The ``cv2.VideoCapture`` object.
    :param camNum: The camera number.
    :param numImages: The number of calibration images to take.
    :param dirPath: Should be aquired by running ``Path(__file__).absolute().parent.__str__()`` in the script calling this method.
    """
    def __init__(self, cap, camNum: int, numImages: int = 15, dirPath: str = "/home/robolions/Documents/2023-Jetson-Code-Test") -> None:
        """
        Constructor for the Calibrate class.
        :param cap: The ``cv2.VideoCapture`` object.
        :param camNum: The camera number.
        :param numImages: The number of calibration images to take.
        :param dirPath: Should be aquired by running ``Path(__file__).absolute().parent.__str__()`` in the script calling this method.
        """
        # Localizes parameters
        self.cap               = cap
        self.camNum            = camNum
        self.calibrationImages = numImages

        # Get height and width
        self.width  = int(self.cap.get(cv.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv.CAP_PROP_FRAME_HEIGHT))

        # Prepare object points
        self.objp = np.zeros((1, CHESSBOARD[0] * CHESSBOARD[1], 3), np.float32)
        self.objp[0, :, :2] = np.mgrid[0:CHESSBOARD[0], 0:CHESSBOARD[1]].T.reshape(-1, 2) * 27.5

        # Arrays to store object and image points from all images
        self.objPoints = []  # 3D point in real world
        self.imgPoints = []  # 2D point in image plane

        # Path to calibration images
        self.PATH = dirPath + f"/camera{self.camNum}-{self.width}x{self.height}-images/"

        # File extension
        self.EXTENSION = ".png"

        # Variables
        self.doOver    = False
        self.logStatus = False

        # Updates log
        Logger.logInfo("Calibration initialized for camera " + str(camNum), True)

    def calibrateCamera(self):
        """
        Calibrates the given camera at a certain resolution.

        :return: Calibration successful.
        :return: The intrinsic camera matrix.
        :return: The camera's distortion.
        :return: The camera's rotation vectors.
        :return: The camera's translation vectors.
        """
        # Variable to see how many images were used in the calibration
        imagesUsed = 0
        self.doOver == False

        # Checks if reference images exist
        refExists = self.getPathExistance()

        # Creates reference images for this resolution if they do not exist or do not contain sufficient data
        if ((refExists == False) or (self.doOver == True)):
            self.createCalibrationImages()
        else:
            pass

        # Gets the path for all the images saved for this camera at a certain resolution
        images = glob.glob(self.PATH + "*" + self.EXTENSION)

        # Loops through stored images
        for image in images:
            # Reads the created images
            img = cv.imread(image)

            # Converts images to grayscale
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

            # Finds chessboard corners. ret is true only if the desired number of corners are found
            ret, corners = cv.findChessboardCorners(gray, CHESSBOARD, cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_FAST_CHECK + cv.CALIB_CB_NORMALIZE_IMAGE)

            # Desired number of corners found, add object and image points after refining them
            if (ret == True):
                # Adds the objectpoint
                self.objPoints.append(self.objp)

                # Refining pixel coordinates for given 2d points.
                corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

                # Adds image point
                self.imgPoints.append(corners2)

                # Draw the corners onto the image
                img = cv.drawChessboardCorners(img, CHESSBOARD, corners2, ret)

                # Increments the imagesUsed count
                imagesUsed += 1

            # Display the image and wait for half a second
            if ((ret == True) & (refExists == False)):
                # Flips the image for easier viewing
                img = cv.flip(img, 1)

                # Displays image
                cv.imshow("img", img)
                cv.waitKey(500)

        # Destroys all cached windows
        cv.destroyAllWindows()

        # Restarts the calibration if 1/2 of the images cannot be used for calibration
        if (imagesUsed < (self.calibrationImages * 1/2)):
            # Updates log
            Logger.logWarning("Calibration restarted", self.logStatus)
            Logger.logInfo(f"Images found: {imagesUsed}", self.logStatus)

            # Sets up for a do over
            self.doOver = True

            self.calibrateCamera()
        else:
            Logger.logInfo(f"Images found: {imagesUsed}", self.logStatus)
            self.doOver = False

        # Calibrate the camera by passing the value of known 3D points (objPoints) and corresponding pixel coordinates of the detected corners (imgPoints)
        self.ret, self.cameraMatrix, self.distortion, self.rVecs, self.tVecs = cv.calibrateCamera(self.objPoints, self.imgPoints, gray.shape[::-1], None, None)

        # Calculates the reprediction error
        repredictError = self.calculateRepredictionError()

        # Updates log
        Logger.logInfo(f"Camera {self.camNum} Calibrated", self.logStatus)

        # Return calibration results
        return self.ret, self.cameraMatrix, self.distortion, self.rVecs, self.tVecs

    def createCalibrationImages(self):
        """
        Creates a number of images to calibrate the camera.
        """
        # Variables
        imgSelected = False

        # Creates the calibration images
        for i in range(0, self.calibrationImages):
            # Seperates the enumeration from the naming
            j = i + 1

            # Prints target image
            print(f"Attempting to take calibration image {j}")

            # Resets imgSelected
            imgSelected = False

            # Runs until the user presses p to take a picture
            while (imgSelected == False):
                # Read the capture
                sucess, stream = self.cap.read()

                # Stores a totally seperate copy of stream
                tempStream = cv.cvtColor(stream, cv.COLOR_BGR2RGB)

                # Converts images to grayscale
                gray = cv.cvtColor(stream, cv.COLOR_BGR2GRAY)

                # Finds chess board corners. ret is true if the desired number of corners are found
                ret, corners = cv.findChessboardCorners(gray, CHESSBOARD, cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_FAST_CHECK + cv.CALIB_CB_NORMALIZE_IMAGE)

                # Draws the chessboard pattern onto the stream
                if (ret == True):
                    # Refining pixel coordinates for given 2d points.
                    corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

                    # Draw the corners onto the stream
                    cv.drawChessboardCorners(stream, CHESSBOARD, corners2, ret)

                # Flips the capture for display purposes
                flippedStream = cv.flip(stream, 1)

                # Display the capture
                cv.imshow("Calibration", flippedStream)

                # Press p to take a calibration image
                if ( cv.waitKey(1) == ord("p") ):
                    # Converts the copy back into BGR
                    tempStream = cv.cvtColor(tempStream, cv.COLOR_RGB2BGR)

                    # Writes the image
                    cv.imwrite(self.PATH + f"{j}" + self.EXTENSION, tempStream)

                    # Prints the status
                    print(f"Calibration image {j} taken")

                    # Updates log
                    Logger.logInfo(f"Calibration image {j} taken", self.logStatus)

                    # Breaks the while loop
                    imgSelected = True
                    break

        # Destroys all windows
        cv.destroyAllWindows()

        # Updates log
        Logger.logInfo("Calibration images stored at " +  self.PATH, self.logStatus)

    def getPathExistance(self) -> bool:
        """
        Gets the existance of a directory at self.PATH.

        :return: Is the directory present?
        """
        # Variables
        img = None
        pathExists = False

        # Attempts to make a directory at self.PATH
        try:
            os.mkdir(self.PATH)
        except Exception as e:
            Logger.logError(f"{e}", self.logStatus)

        # Attempts to read the last callibration image and updates variables accordingly
        img = cv.imread(self.PATH + str(self.calibrationImages) + self.EXTENSION)

        # Determines if the calibration imgaes exist
        if (img is not None):
            pathExists = True
            Logger.logInfo("PATH already exists", self.logStatus)
            print("PATH already exists")
        else:
            pathExists = False
            Logger.logWarning("PATH does not exist", self.logStatus)
            print("PATH does not exist")

        return pathExists

    def calculateRepredictionError(self):
        """
        Calulates the reprediction error.

        :return: The mean reprediction error.
        """
        # Reprediction error
        total_error = 0

        # Loops through object points
        for i in range(len(self.objPoints)):
            imgPoints2, _ = cv.projectPoints(self.objPoints[i], self.rVecs[i], self.tVecs[i], self.cameraMatrix, self.distortion)
            error = cv.norm(self.imgPoints[i], imgPoints2, cv.NORM_L2) / len(imgPoints2)
            total_error += error

        # Calculates mean error from the total
        mean_error = total_error / len(self.objPoints)

        # Returns the mean error
        return mean_error

    def enableLogging(self):
        """
        Enables logging for this class.
        """
        self.logStatus = True