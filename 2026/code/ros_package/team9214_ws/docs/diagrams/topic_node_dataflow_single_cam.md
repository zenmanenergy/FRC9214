/camera_1/image_raw  +  /camera_1/camera_info
          |                     |
          v                     v
     apriltag_detector_node (camera_1)
                     |
                     v
              /camera_1/tag_detections   (tag pose in camera_1_optical_frame)
                     |
                     v
 tag_pose_observer_node
   - loads arena_tags.yaml (T_map_tag per ID)
   - reads TF (T_base_cam_1)
   - computes T_map_base observation
          |
          v
   /tag_global_pose   (PoseWithCovarianceStamped in map)
          |
          v
 robot_localization (ekf_map)
   - fuses wheel/imu odom with global pose
   - publishes map->odom TF
