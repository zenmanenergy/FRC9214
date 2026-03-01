import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Example of loading parameters from a YAML file
    # You would need to create a config/params.yaml file in your package
    # params_file = os.path.join(
    #     get_package_share_directory('your_package_name'),
    #     'config',
    #     'params.yaml'
    # )

    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            # name='usb_cam',
            name='camera',
            output='screen',
            parameters=[
                {'video_device': '/dev/video0'}, # Check your device path
                {'image_width': 640},
                {'image_height': 480},
                # {'image_width': 1280},
                # {'image_height': 720},
                # {'framerate': 15.0}, # Start with a lower framerate for calibration
                {'framerate': 30.0}, # Note: framerate is a double in ROS2
                # {'camera_info_url': 'file:///home/robots/source_code/first_robotics_comp/frc/camera_ws/src/honking_narwhal/calib/camera2.yaml'} # Add this after calibratio
            ],
            # The calibration tool looks for topic names that differ 
            # from the default topic names published by the `usb_cam` 
            # driver.  The mapping below handles mapping default 
            # topics to the topics needed by the calibration tool.
            remappings=[
                ('image_raw', '/camera/image_raw'),
                ('camera_info', '/camera/camera_info'),
            ]
        ),
    ])
