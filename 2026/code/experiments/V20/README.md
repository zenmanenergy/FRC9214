# FRC 9214 Swerve Drive Robot

Welcome! This is the code for Team 9214's swerve drive robot. This robot can move in any direction and rotate at the same time—something most wheeled robots can't do.

## What Does This Robot Do?

This is a **swerve drive** robot used for FRC (FIRST Robotics Competition). It's designed to move around the game field with precision and speed. The key features are:

- **Moves in any direction** - It can drive forward, sideways, or at an angle without turning first
- **Rotates while moving** - It can spin and move at the same time
- **Follows automatic paths** - In autonomous mode, it can follow pre-planned paths across the field
- **Precise control** - We can fine-tune its movement through calibration and tuning

## Getting Started

If you're new to the codebase, start here:

1. **[Swerve Drive Explained](docs/Swerve-Drive-Explained.md)** - Understand how swerve drive works and why it's special
2. **[Simulator Guide](docs/Simulator-Guide.md)** - Learn how to test the robot using simulation and the dashboard
3. **[Autonomous Paths](docs/Autonomous-Paths.md)** - See how the robot plans and follows paths on its own

## What's in This Folder?

```
robot.py                    - Main robot code (startup point)
swerve/                     - Swerve drive system (motors, wheels, control)
dashboard/                  - Web interface for testing and tuning
tests/                      - Testing and calibration code
pilot_controls.py           - Joystick input handling
waypoint_navigator.py       - Autonomous path following
```

## Running the Robot

The robot runs through the standard FRC deployment process. Check with your team lead for deployment instructions.

---

**Questions?** Ask a team member or check the docs folder for more detailed guides.
