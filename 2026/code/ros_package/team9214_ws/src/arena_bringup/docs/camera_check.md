## System Camera Check

Steps to setup the camera and verify the video stream can be accessed.

### Dependencies

```bash
apt-get update && apt-get install -y --no-install-recommends \
  v4l-utils \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  && rm -rf /var/lib/apt/lists/*
```

### Check V4L2 sees it:

`v4l2-ctl --list-devices`

#### You should see something like

```bash
root@robots:/# v4l2-ctl --list-devices
CREALITY CAM: CREALITY CAM (usb-0000:00:14.0-1):
	/dev/video0
```

### Additional Check

`v4l2-ctl -d /dev/video0 --all`

#### Output example

```bash
root@robots:/# v4l2-ctl -d /dev/video0 --all
Driver Info:
	Driver name      : uvcvideo
	Card type        : CREALITY CAM: CREALITY CAM
	Bus info         : usb-0000:00:14.0-1
	Driver version   : 6.14.11
	Capabilities     : 0x84a00001
		Video Capture
		Metadata Capture
		Streaming
		Extended Pix Format
		Device Capabilities
	Device Caps      : 0x04200001
		Video Capture
		Streaming
		Extended Pix Format
Priority: 2
Video input : 0 (Camera 1: ok)
Format Video Capture:
	Width/Height      : 1920/1080
	Pixel Format      : 'MJPG' (Motion-JPEG)
	Field             : None
	Bytes per Line    : 0
	Size Image        : 4147200
	Colorspace        : sRGB
	Transfer Function : Default (maps to sRGB)
	YCbCr/HSV Encoding: Default (maps to ITU-R 601)
	Quantization      : Default (maps to Full Range)
	Flags             : 
Crop Capability Video Capture:
	Bounds      : Left 0, Top 0, Width 1920, Height 1080
	Default     : Left 0, Top 0, Width 1920, Height 1080
	Pixel Aspect: 1/1
Selection Video Capture: crop_default, Left 0, Top 0, Width 1920, Height 1080, Flags: 
Selection Video Capture: crop_bounds, Left 0, Top 0, Width 1920, Height 1080, Flags: 
Streaming Parameters Video Capture:
	Capabilities     : timeperframe
	Frames per second: 30.000 (30/1)
	Read buffers     : 0

User Controls

                     brightness 0x00980900 (int)    : min=0 max=255 step=1 default=128 value=128
                       contrast 0x00980901 (int)    : min=0 max=255 step=1 default=128 value=128
                     saturation 0x00980902 (int)    : min=0 max=255 step=1 default=128 value=128
        white_balance_automatic 0x0098090c (bool)   : default=1 value=1
                           gain 0x00980913 (int)    : min=0 max=100 step=1 default=5 value=5
           power_line_frequency 0x00980918 (menu)   : min=0 max=2 default=2 value=1 (50 Hz)
				0: Disabled
				1: 50 Hz
				2: 60 Hz
      white_balance_temperature 0x0098091a (int)    : min=0 max=255 step=1 default=128 value=128 flags=inactive
                      sharpness 0x0098091b (int)    : min=0 max=255 step=1 default=128 value=128

Camera Controls

                  auto_exposure 0x009a0901 (menu)   : min=0 max=3 default=0 value=0 (Auto Mode)
				0: Auto Mode
				1: Manual Mode
				2: Shutter Priority Mode
				3: Aperture Priority Mode
         exposure_time_absolute 0x009a0902 (int)    : min=0 max=6500 step=1 default=100 value=100 flags=inactive
```

#### Smoke-test stream with GStreamer (no GUI required)

```bash
# If you have no display, just ensure frames flow (prints running pipeline)
gst-lgaunch-1.0 v4l2src device=/dev/video0 ! videoconvert ! fakesink
```

##### Smoke-test output

```bash
root@robots:/# gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! fakesink
Setting pipeline to PAUSED ...
Pipeline is live and does not need PREROLL ...
Pipeline is PREROLLED ...
Setting pipeline to PLAYING ...
New clock: GstSystemClock
Redistribute latency...
handling interrupt.
Interrupt: Stopping pipeline ...
Execution ended after 0:00:24.528033822
Setting pipeline to NULL ...
Freeing pipeline ...
```
