# Created by Alex Pereira

# Import Libraries
import time
import logging

# Start of the Logging class
class Logger:
    """
    Use this class to log an debug info you generate.
    """
    @staticmethod
    def setLogPath(dirPath: str = "/tmp/"):
        """
        Enables the logger.

        :param dirPath: Should be aquired by running ``Path(__file__).absolute().parent.__str__()`` in the script calling this method
        """
        # Gets the time as a string
        currTime = time.ctime().replace(" ", "_")[4:]

        # Starts a logger
        logging.basicConfig(filename = dirPath + f"/{currTime}.log", format="%(levelname)s:%(message)s", encoding = "utf-8", level = logging.DEBUG)

    @staticmethod
    def logDebug(debug: str, logStatus: bool = True):
        """
        Logs a debug statement.

        :param debug: The debug message to log
        :param logStatus: Is logging enabled
        """
        if (logStatus == True):
            logging.debug(debug)

    @staticmethod
    def logInfo(info: str, logStatus: bool = True):
        """
        Logs information.

        :param info: The info message to log
        :param logStatus: Is logging enabled
        """
        if (logStatus == True):
            logging.info(info)

    @staticmethod
    def logWarning(warning: str, logStatus: bool = True):
        """
        Logs a warning.

        :param warning: The warning message to log
        :param logStatus: Is logging enabled
        """
        if (logStatus == True):
            logging.warning(warning)

    @staticmethod
    def logError(error: str, logStatus: bool = True):
        """
        Logs an error.

        :param error: The error message to log
        :param logStatus: Is logging enabled
        """
        if (logStatus == True):
            logging.error(error)