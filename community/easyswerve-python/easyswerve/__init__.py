"""
easyswerve: the EasySwerve swerve drive library.

Students use two things from it, in robot.py:

    from easyswerve import Drivetrain, Autos

Robot-specific values go in robot_settings.py. Everything in this folder is
library code; coaches can find the details in docs/.
"""

__all__ = ["Drivetrain", "Autos"]


def __getattr__(name):
    # Loaded on first use, not at import. robot_settings.py imports
    # easyswerve.settings_types, and the library reads robot_settings, so
    # loading everything here up front would make the two import each other.
    if name == "Drivetrain":
        from .drivetrain.drivetrain import Drivetrain

        return Drivetrain
    if name == "Autos":
        from .autos.autos import Autos

        return Autos
    raise AttributeError(f"module 'easyswerve' has no attribute {name!r}")
