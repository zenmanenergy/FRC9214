camera 1

*** Calibrating ****
mono pinhole calibration...
D = [-0.3358467439702639, 0.08654598384510657, -0.004380958919683536, -0.0008103906419789178, 0.0]
K = [467.4921085526066, 0.0, 312.95542759492986, 0.0, 628.8896141202166, 251.14307512081376, 0.0, 0.0, 1.0]
R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P = [373.9935302734375, 0.0, 306.95757447600045, 0.0, 0.0, 594.8986206054688, 250.43315610558238, 0.0, 0.0, 0.0, 1.0, 0.0]
None
# oST version 5.0 parameters


[image]

width
640

height
480

[narrow_stereo]

camera matrix
467.492109 0.000000 312.955428
0.000000 628.889614 251.143075
0.000000 0.000000 1.000000

distortion
-0.335847 0.086546 -0.004381 -0.000810 0.000000

rectification
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000

projection
373.993530 0.000000 306.957574 0.000000
0.000000 594.898621 250.433156 0.000000
0.000000 0.000000 1.000000 0.000000



Robot Camera:

robots@robots:~$ v4l2-ctl -d /dev/video4 --list-ctrls

User Controls

                     brightness 0x00980900 (int)    : min=0 max=255 step=1 default=128 value=50 flags=0x00001000
                       contrast 0x00980901 (int)    : min=0 max=255 step=1 default=128 value=128 flags=0x00001000
                     saturation 0x00980902 (int)    : min=0 max=255 step=1 default=128 value=128 flags=0x00001000
        white_balance_automatic 0x0098090c (bool)   : default=1 value=1
                           gain 0x00980913 (int)    : min=0 max=100 step=1 default=5 value=5 flags=0x00001000
           power_line_frequency 0x00980918 (menu)   : min=0 max=2 default=2 value=1 (50 Hz)
      white_balance_temperature 0x0098091a (int)    : min=0 max=255 step=1 default=128 value=128 flags=inactive, 0x00001000
                      sharpness 0x0098091b (int)    : min=0 max=255 step=1 default=128 value=128 flags=0x00001000

Camera Controls

                  auto_exposure 0x009a0901 (menu)   : min=0 max=3 default=0 value=0 (Auto Mode)
         exposure_time_absolute 0x009a0902 (int)    : min=0 max=6500 step=1 default=100 value=100 flags=inactive, 0x00001000

Alpha Webcam

robots@robots:~$ v4l2-ctl -d /dev/video2 --list-ctrls

User Controls

                     brightness 0x00980900 (int)    : min=-64 max=64 step=1 default=0 value=50 flags=0x00001000
                       contrast 0x00980901 (int)    : min=0 max=100 step=1 default=50 value=50 flags=0x00001000
                     saturation 0x00980902 (int)    : min=0 max=100 step=1 default=64 value=64 flags=0x00001000
                            hue 0x00980903 (int)    : min=-180 max=180 step=1 default=0 value=0 flags=0x00001000
        white_balance_automatic 0x0098090c (bool)   : default=1 value=1
                          gamma 0x00980910 (int)    : min=100 max=500 step=1 default=300 value=300 flags=0x00001000
           power_line_frequency 0x00980918 (menu)   : min=0 max=2 default=2 value=2 (60 Hz)
      white_balance_temperature 0x0098091a (int)    : min=2800 max=6500 step=10 default=4600 value=4600 flags=inactive, 0x00001000
                      sharpness 0x0098091b (int)    : min=0 max=100 step=1 default=80 value=80 flags=0x00001000
         backlight_compensation 0x0098091c (int)    : min=0 max=2 step=1 default=1 value=1 flags=0x00001000

Camera Controls

                  auto_exposure 0x009a0901 (menu)   : min=0 max=3 default=3 value=3 (Aperture Priority Mode)
         exposure_time_absolute 0x009a0902 (int)    : min=50 max=10000 step=1 default=166 value=166 flags=inactive, 0x00001000
     exposure_dynamic_framerate 0x009a0903 (bool)   : default=0 value=1
                   pan_absolute 0x009a0908 (int)    : min=-57600 max=57600 step=3600 default=0 value=0 flags=0x00001000
                  tilt_absolute 0x009a0909 (int)    : min=-43200 max=43200 step=3600 default=0 value=0 flags=0x00001000
                  zoom_absolute 0x009a090d (int)    : min=0 max=3 step=1 default=0 value=0 flags=0x00001000
