cam_i detections -> observer_i -> /tag_pose_obs/cam_i
                                     |
                                     v
                         pose_fuser_node (cov-weight, gating)
                                     |
                                     v
                              /tag_global_pose
