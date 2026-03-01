# Camera Calibration with ROS2

To generate reliable `rectification/pose` you will need to have a valid calibration YAML so that `camera_info` is non-zero The common ROS2 path is `camera_calibration` (checkerboard) which writes a camera calibration YAML. The following will walk through how to first generate a configuration file with a single camera. 

## Monocular Calibration

Calibration of a single camera

### Workflow

- Camera Driver &rarr; calibration tool &rarr; calibration file

### Install calibration tool

```bash
apt-get update \
    && apt-get install -y \
    ros-${ROS_DISTRO}-camera-calibration
    ros-${ROS_DISTRO}-image-pipeline
```


### Create launch file if not already created

```bash
cd src/arena_bringup/launch/usb_cam_calibration.launch.py
```

#### Map topics

- The calibration tool is looking for images on `/camera/image_raw` and info on `/camera/camera_info` so we need to map the `usb_cam` topics as follows:
    ```bash
    remappings=[
        ('image_raw', '/camera/image_raw'),
        ('camera_info', '/camera/camera_info'),
    ```

### Terminal 1 - Launch the usb camera 

```bash
cd src/arena_bringup/launch
ros2 launch usb_cam_calibration.launch.py
```

<!-- 
```bash
# temporarily: comment out rectify/apriltag/classifier in the launch, or run only usb_cam
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file config/usb_cam.yaml
``` -->

### Terminal 2 - Check service and topics

We want to check for 

1. Node: `/camera`

```bash
team9214_ws$ ros2 node list
/camera
```

1. Service : `/camera/set_camera_info`

```bash
team9214_ws$ ros2 service list
/camera/describe_parameters
/camera/get_parameter_types
/camera/get_parameters
/camera/get_type_description
/camera/list_parameters
/camera/set_camera_info
/camera/set_parameters
/camera/set_parameters_atomically
/set_capture

```

3. Topics: `/camera/image_raw` and `/camera/image_raw`

```bash
team9214_ws$ ros2 topic list
/camera/camera_info
/camera/image_raw
/image_raw/compressed
/image_raw/compressedDepth
/image_raw/theora
/image_raw/zstd
/parameter_events
/rosout
```

4. Frequency of camera images is around 30

`ros2 topic hz /camera/image_raw`

```bash
team9214_ws$ ros2 topic hz /camera/image_raw 
average rate: 29.953
	min: 0.031s max: 0.039s std dev: 0.00217s window: 31
average rate: 29.975
	min: 0.031s max: 0.039s std dev: 0.00216s window: 61
average rate: 29.981
	min: 0.030s max: 0.039s std dev: 0.00215s window: 91
```

*Note* The node name comes from `name` field in the `node` section of the launch file. The calibration tool is looking for a node `name=camera`. We originally had this as `name='usb_cam'` and the calibration tool was not seeing the service. We changed this to `name='camera'` to get the correct serv ice name needed by the calibration tool.

### Terminal 2 - Run calibration tool

`ros2 run camera_calibration cameracalibrator --size 7x9 --square 0.020 --ros-args -r image:=/camera/image_raw`

*note* There is other command line parameters that can be passed if you need to override any other defaults

1. Move board around the FOV of the camera

You will see colored lines on the squares.  I find it useful to have the terminal visable on part of the screen and the GUI showing the image on another part.  Once enough data is obtained the `Calibrate` botton will enable.  Press it and there will be a terminal output with the calibration info.  You can also press `Save` and a tarball will be generated.  The tool will indicate where it is saved: `('Wrote calibration data to', '/tmp/calibrationdata.tar.gz')` Extract the tarball and cp the `ost.yaml` version of the configuration files.

2. Save the produced YAML into package:

```bash
cp ost.yaml 
src/mutli_cam_apriltag/calib/<useful-camera-name>.yaml
```

2. Update the `camera_info_url` in the `team9214_ws/src/arena_bringup/config/bringup.yaml` and 

Ensure `camera_info_url` `file:////home/robots/source_code/github/zenmanenergy/FRC9214/2026/code/ros_package/team9214_ws/src/multi_cam_apriltag/config/AlphaCam_calibration.yaml` stays set (URL format matters). 

## Calibration Board

We are using the standard 8x11 paper sized checkerboard with `8x10` with `20mm` squaresfirst described above for these instruction.

We generated a board that was `8x10` `20mm` squares.  The calibration tool needs a have this defined as `7x9`.  The tool needs the inner edge of the first set of squares as the boundary. To summarize, if there is a `NxM` square created the tool needs to have the square defined as `(N-1)x(M-1)`

Calibration boards are needed for both single and stereo calibration. I'm sure there is other types to use and I do not know which is best for what, but I have used a checkerboard pattern in my past work so will stick with that pattern for these instructions. The are located in `docs/calibration_boards` but can also be regenerated at the following:

`https://calib.io/pages/camera-calibration-pattern-generator`

### Sizes

#### 8x10 Checkerboard with 20m squares

#### 8x10 Checkerboard with 15mm squares

`camera_ws/src/honking_narwhals/docs/calibration_boards/calib.io_checker_200x150_8x10_15.pdf`

```text
Target Type: Checkerboard
Board Width: 200 mm
Board Height: 150 mm
Rows: 8
Columns: 10
Checker Width: 15mm
```

#### 7x9 checkerboard with 15mm squares

`honking_narwhal/docs/calibration_boards/calib.io_checker_200x150_7x9_15.pdf`

```text
Target Type: Checkerboard
Board Width: 200 mm
Board Height: 150 mm
Rows: 7
Columns: 9
Checker Width: 15mm
```

### Run monocular calibration

`ros2 run camera_calibration cameracalibrator --size 6x8 --square 0.015 --ros-args -r image:=/camera/image_raw`
