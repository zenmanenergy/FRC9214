# FRC Team 9214 - 2026 Season Repository

## About the Honking Narwhals

The **Honking Narwhals** are a FIRST Robotics Competition (FRC) team based in Norfolk, Virginia. We operate out of [757 Makerspace](http://www.757makerspace.org/) and compete annually in the FRC challenge.

**Team Website:** [honkingnarwhals.com](http://www.honkingnarwhals.com)

## Repository Overview

This folder contains all code and CAD design files for the Honking Narwhals' 2026 robot platform. The repository is organized to support both mechanical design (SolidWorks CAD files) and software development (Python-based robot control code).

## Folder Structure

### 📁 `/CAD` - Mechanical Design Files

Contains all SolidWorks CAD files for the 2026 robot design:

- **`2026 ROBOT/`** - Main robot assembly and components
  - `_9214_V2_tank_drive.SLDASM` - Tank drive base assembly
  - `tank base/` - Frame and drive structure
  - `elevator/` - Vertical lift mechanism
  - `upper arm/` & `lower arm/` - Multi-jointed arm system
  - `vertical rails/` - Rail systems for linear motion
  - `cameras/` - Camera mount systems
  - `sidewall/` - Side panel components
  - `shared/` - Shared/common parts used across assemblies
  - `Drawings/` - Engineering drawings and documentation
  - `FTC Intake Device.SLDPRT` - Game piece intake mechanism

- **`ToolBox/`** - Tools and utilities for design

### 📁 `/code` - Robot Software

Python-based software for robot control and experimentation:

- **`experiments/`** - Experimental code and prototypes
  - `V1 odometry/` - Odometry calculations and position tracking (Version 1)
  - `V2 ROS helloworld/` - ROS (Robot Operating System) integration experiments
  - `V3 GUI hello world/` - GUI/dashboard interface prototypes
  - `V4 GUI Odometry/` - Advanced GUI with odometry integration
  - `V5 Gyro/` - Gyroscope sensor integration
  - `V6 Lidar/` - LIDAR sensor integration and processing

## Getting Started

### Prerequisites

- Python 3.x
- WPILib (FRC Python library)
- Any specific dependencies listed in individual experiment folders

### Running Code

1. Navigate to the desired experiment version folder
2. Install any required dependencies
3. Run the main robot code file

### CAD Design

All CAD files are created using **SolidWorks**. To view or edit:
1. Open files with SolidWorks 2020 or later
2. Main assembly: `CAD/2026 ROBOT/_9214_V2_tank_drive.SLDASM`

## Team Information

- **Team Number:** 9214
- **Location:** Norfolk, Virginia
- **Workspace:** 757 Makerspace
- **Competition:** FIRST Robotics Competition (FRC)

## Repository Structure

This is part of a larger FRC9214 repository that also contains:
- Historical code and designs from previous seasons (2023, 2025)
- Vision processing (AprilTags, stereocameras)
- Path-finding algorithms
- Dashboard and GUI frameworks
- Student-led experiments and learning projects

For information about other seasons, see the parent directory README.

## Contributing

This is the official repository for the Honking Narwhals FRC Team. 

- **Code contributions** should follow Python best practices
- **CAD files** should maintain the folder structure and naming conventions
- **Documentation** should be kept up-to-date with code changes

## License

This repository contains design and code for the Honking Narwhals FRC Team 9214. Usage rights and licensing information can be found in the parent repository.

---

**Last Updated:** 2026 Season  
**Contact:** Visit [honkingnarwhals.com](http://www.honkingnarwhals.com) for team information
