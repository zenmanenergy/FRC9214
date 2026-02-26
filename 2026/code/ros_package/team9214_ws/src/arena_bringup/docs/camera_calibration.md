# Camera Calibration with ROS2

To generate reliable `rectification/pose` you will need to have a valid calibration YAML so that `camera_info` is non-zero The common ROS2 path is `camera_calibration` (checkerboard) which writes a camera calibration YAML. The following will walk through how to first generate a configuration file with a single camera. 

## Calibration Board

Calibration boards are needed for both single and stereo calibration. I'm sure there is other types to use and I do not know which is best for what, but I have used a checkerboard pattern in my past work so will stick with that pattern for these instructions. The are located in `docs/calibration_boards` but can also be regenerated at the following:

`https://calib.io/pages/camera-calibration-pattern-generator`

### Sizes

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


## Monocular Calibration

Calibration of a single camera

### Workflow

- Camera Driver &rarr; calibration tool &rarr; calibration file

### Install calibration tool

`apt-get update && apt-get install -y ros-jazzy-camera-calibration`

### Create launch file if not already created

`launch/usb_cam_cal.launch.py`

### Launch the camera only setup we just created

`ros2 launch usb_cam_cal.launch.py`

<!-- 
```bash
# temporarily: comment out rectify/apriltag/classifier in the launch, or run only usb_cam
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file config/usb_cam.yaml
``` -->

### Check service and topics

#### Service

```bash
robots@robots:~/source_code/first_robotics_comp/frc$ ros2 service list
/set_capture
/camera/describe_parameters
/camera/get_parameter_types
/camera/get_parameters
/camera/get_type_description
/camera/list_parameters
/camera/set_camera_info
/camera/set_parameters
/camera/set_parameters_atomically
```

#### Topics

```bash
robots@robots:~/source_code/first_robotics_comp/frc$ ros2 topic list
/camera/image_raw
/camera/set_camera_info
/image_raw/compressed
/image_raw/compressedDepth
/image_raw/theora
/image_raw/zstd
/parameter_events
/rosout
```

*Note* The `camera` string in the service name comes from the `name=""` pair in the launch file.  We originally had this as `name='usb_cam'` and the calibration tool was not seeing the service. We changed this to `name='camera'` to get the correct service name needed by the calibration tool.

### Setup calibration checkerboard

We are using the standard 8x11 paper sized checkerboard with `7x9` with `15mm` squaresfirst described above for these instruction.  We may need to change this to a larger board for stereo calibration to work well but we will not know until we see the results.

We generated a board that was `7"x9"` with `15mm` squares.  The calibration tool needs a have this defined as `6x8`.  The tool needs the outer edge of the first set of inner square edges.  To summarize, if there is a `NxM` square created the tool needs to have the square defined as `(N-1)x(M-1)`

### Run monocular calibration

`ros2 run camera_calibration cameracalibrator --size 6x8 --square 0.015 --ros-args -r image:=/camera/image_raw`

### Move board around the FOV of the camera

You will see colored lines on the squares.  Once enough data is obtained the `Calibrate` botton will enable.  Press it and there will be a terminal output with the calibration info.  You can also press `Save` and you will get a tarball with the images an `ini` and `yaml` version of the configuration files.

### Save the produced YAML into:

`calib/usb_cam.yaml`

### Check camera_info_url

Ensure `camera_info_url` `file:///work/calib/usb_cam.yaml` stays set (URL format matters). 
