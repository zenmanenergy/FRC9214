
# Simplified project build instructions

## Make sure ROS2 is installed on host OS

### Additional packages that may be needed

`sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup`
`sudo apt install ros-jazzy-nav2-*`

## Ensure you have python 3 and venv support:

```bash
python3 --version
python3 -m venv -h >/dev/null
```

Install pip if not installed

`sudo apt-get install -y python3-venv python3-pip`

---

## Create + activate a venv

### Clean up old python if present

```bash
rm -rf build install log
rm -rf .venv
pip cache purge
```

### Create + activate

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
```

---

## Python Prereqs in venv

```bash
python -m pip install --upgrade pip setuptools wheel pyyaml jinja2 typeguard lark
python -m pip install pyntcore
```

---

## Source ROS

`source /opt/ros/jazzy/setup.bash`

### Verify can see ntcore, rcply, and *msgs packages

```bash
python -c "import ntcore; print('ntcore OK', ntcore.__file__)"
python -c "import rclpy; print('rclpy OK', rclpy.__file__)"
python -c "from geometry_msgs.msg import AccelWithCovariance; print('geometry_msgs OK')"
```

### Output of verify check

```bash
(.venv) usv@usv:~/src/github/sidenoteemail/frc/ws$ python -c "import ntcore; print('ntcore OK', ntcore.__file__)"
ntcore OK /home/usv/src/github/sidenoteemail/frc/ws/.venv/lib/python3.12/site-packages/ntcore/__init__.py
(.venv) usv@usv:~/src/github/sidenoteemail/frc/ws$ python -c "import rclpy; print('rclpy OK', rclpy.__file__)"
rclpy OK /opt/ros/jazzy/lib/python3.12/site-packages/rclpy/__init__.py
(.venv) usv@usv:~/src/github/sidenoteemail/frc/ws$ python -c "from geometry_msgs.msg import AccelWithCovariance; print('geometry_msgs OK')"
geometry_msgs OK
```

---

## Create blank ROS2 package called `team9214` with `ros_to_nt`

```bash
mkdir -p ~/ws/src/
cd ~/ws/src
ros2 pkg create --build-type ament_python --node-name ros_to_nt team9214 --dependencies rclpy geometry_msgs nav_msgs pyntcore
touch team9214/team9214/nt_to_ros.py
mkdir -p team9214/launch
touch team9214/launch/team9214.launch.py
```
**NOTE** I have not found a way to add multiple nodes when creating a package

### Populate files as needed
- Populate python node files as necessary
    - Python nodes need to be located at `ws/src/team9214/team9214`
- Update `package.xml` if needed
- Update `setup.py`
- Populate `team9214.launch.py`

### Build the package

```bash
cd /home/usv/src/github/sidenoteemail/frc/ws/
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install
source install/setup.bash
```

## Run ROS package

**IMPORTANT** The RoboRIO must be setup if you expect to see data coming into the package.

`ros2 launch team9214 team9214.launch.py`

### Run plotter if useful

File located at: `~/ws/src/team9214/team9214/shoreside_odom_plotter.py`

Run:

`python3 ~/ws/src/team9214/team9214/shoreside_odom_plotter.py --cmd-topic /cmd_vel --odom-topic /odom --csv /tmp/run.csv`

---

## RoboRIO setup

---

### Install robotpy

- Make sure we are in the correct venv

```bash
source ~/ws/.venv/bin/activate
which python
```

- Install robotpy

`python3 -m pip install robotpy`

If just need to upgrade:

        `python3 -m pip install --upgrade robotpy`

---

### Sync the RoboRIO

- Run `sync` subcommand to
    - Download `Python` compiled for `roboRIO`
    - Download `roboRIO` compatible python packages as specified by your `pyproject.toml`
    - Install the packages specified by your `pyproject.toml` into your local environment

    `python -m robotpy sync`

    *note* I uncommented out `all` in the `pyproject.toml` to have all robotpy packages installed to see if could be done.  Probably do not need all of that for `robot.py` logic. We still need to update the `pyproject.toml` to include any additional python package.

### Deploy `robot.py` to the RoboRIO

- Command to deploy to RoboRIO

`python -m robotpy deploy`

*Note:* If the python packages installed on the RoboRIO do not match it will ask if you want them uninstalled and the matching ones installed.  I said YES and it uninstalled and installed for me.

---

### Simulator

RobotPy provides a simulator.  I have not worked much with this yet, but it looks promising

`python -m robotpy sim`

A GUI will pop up.  I will add more instructions here once I have flushed it out.

---

## Python Requirements Tracking

**IMPORTANT** Determine if this works when `venv` is created with `--system-site-packages` flag. Once it works generate `requirements.txt`

`python -m pip freeze | tee requirements.txt`

or

`pip freeze > requirements.txt`

Generated File: `sidenoteemail/frc/ws/requirements.txt`

---

### Moving/provisioning a venv directory/setup to another location

- Create requirements.txt from venv you want to reproduce

```bash
source /path/to/current/.venv/bin/activate
pip freeze > requirments.txt
deactivate
```

- Create new venv in the new location

```bash
python3 -m ~/venv/team9214
source ~/venv/team9214/bin/activate
```

- Reinstall packages

`pip install -r /path/to/saved/requirements.txt`

---

## Network tables Info

- NetworkTables is client–server
    - The RoboRIO runs the NT server (always-on, authoritative).
    - External processes (Driver Station, dashboards, ROS 2 bridges) act as clients.
- Your ROS 2 app is not hosting NT
    - It connects to the RoboRIO
    - It does not accept inbound NT connections
- So your ROS 2 node should be configured like:
    `nt_server = <roborio-ip>`

    ```bash
    +---------------------+         NT Client       +--------------------+
    | ROS 2 Shoreside App |  -------------------->  | RoboRIO            |
    | (ntcore client)     |                         | NT Server (wpilib) |
    +---------------------+                         +--------------------+
    ```

### Verification

- ping should succeed
    `ping <roborio-ip>`

- In code:

```python
from ntcore import NetworkTableInstance

nt = NetworkTableInstance.getDefault()
nt.startClient4("ros2-bridge")
nt.setServer("<roborio-ip>")   # <-- correct
nt.startDSClient()             # optional but recommended
```

---

## NT Topic lists

---

### Clients
- nt_to_ros
    - Subscribers
        - "/SmartDashboard/odom_y_m"
        - "/SmartDashboard/odom_vx_m"
        - "/SmartDashboard/odom_wz_radps"
        - "/SmartDashboard/odom_x_m"
        - "/SmartDashboard/odom_yaw_rad"
        - "/SmartDashboard/odom_vy_mps"

- ros_to_nt
    - Publishers
        - "/SmartDashboard/cmd_vel_vx_mps"
        - "/SmartDashboard/cmd_vel_wz_radps"
        - "/SmartDashboard/cmd_vel_vy_mps"
        - "/SmartDashboard/cmd_vel_stamp_s"


---

## Helpful topics

---

### Check NT4 tables

Run `/home/usv/wpilib/2026/tools/glass`
Config file saved at `/home/usv/src/github/sidenoteemail/frc/frc_tool_configs/glass`

Run `/home/usv/wpilib/2026/tools/outlineviewer`

---

### wpilib python releases

`https://wpilib.jfrog.io/ui/native/wpilib-python-release-2026/`

---

### robotpy documentation
`https://robotpy.readthedocs.io/en/stable/`

`https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html`

`https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html`

---

### robotpy subcommands

`https://docs.wpilib.org/en/stable/docs/software/python/subcommands/index.html`

---

### Test publisher for ROS

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}" -r 10
```

--- 