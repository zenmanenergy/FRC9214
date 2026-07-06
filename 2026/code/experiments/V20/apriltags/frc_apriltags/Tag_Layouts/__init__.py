# Import libraries
from importlib_metadata import PackageNotFoundError, version

# Generate the version
try:
    __version__ = version("frc-apriltags")
except PackageNotFoundError:
    # Package is not installed
    pass