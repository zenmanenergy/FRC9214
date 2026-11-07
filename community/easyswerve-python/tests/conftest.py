"""
Shared test helpers.

py -m robotpy deploy runs these tests on the laptop before every deploy, using
whatever is in robot.py right now: often a setup step or an example. So:
  - tests of the controller and buttons only run with the starting robot.py
    (examples/01_basic, which sets STARTING_ROBOT = True)
  - tests that just need a drivetrain run whenever robot.py builds one
  - everything else is skipped, so it never blocks a deploy
"""

import sys

import pytest

from easyswerve.sim.physics import findDrivetrain


def drivetrainOrSkip(robot):
    """The robot's drivetrain, or skip the test if robot.py has none."""
    drive = findDrivetrain(robot, required=False)
    if drive is None:
        pytest.skip("robot.py builds no drivetrain (a setup step), so there is nothing to test here")
    return drive


def startingRobotOrSkip(robot):
    """The drivetrain, but only if robot.py is the starting robot, whose
    controller buttons these tests know."""
    module = sys.modules.get(type(robot).__module__)
    if not getattr(module, "STARTING_ROBOT", False):
        pytest.skip("controller tests run only when robot.py is examples/01_basic")
    return drivetrainOrSkip(robot)
