           map
            |
            |  (from localization_fusion / robot_localization)
            v
           odom
            |
            |  (from wheel odom fusion / robot_localization or your odom source)
            v
         base_link
          /   |   \
         /    |    \
 camera_1  camera_2  camera_3
   |         |         |
 cam_1_opt  cam_2_opt  cam_3_opt
