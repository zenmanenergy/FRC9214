import math

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros


class TagDistanceReader(Node):
    def __init__(self) -> None:
        super().__init__("tag_distance_reader")
        self.declare_parameter("camera_frame", "camera_1_optical_frame")
        self.declare_parameter("detections_topic", "/camera_1/tag_detections")
        self.declare_parameter("tag_family", "tag36h11")
        self.declare_parameter("lookup_timeout_sec", 0.05)

        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.tag_family = str(self.get_parameter("tag_family").value)
        self.lookup_timeout_sec = float(self.get_parameter("lookup_timeout_sec").value)

        # Setup Transform Listeners
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_tag_id = None
        self.latest_frame_id = ""

        # Subscribe to Detections
        self.create_subscription(
            AprilTagDetectionArray, # Topic
            self.detections_topic,  # Datatype
            self._detections_cb,    # Callback function
            10,                     # 
        )

        # Periodicly call _report_distance function
        self.create_timer(0.2, self._report_distance)

        self.get_logger().info(
            f"Listening on {self.detections_topic}, camera_frame={self.camera_frame}, tag_family={self.tag_family}"
        )

    # Callback called when detections are recieved.
    def _detections_cb(self, msg: AprilTagDetectionArray) -> None:
        if not msg.detections:
            return
        det = msg.detections[0]
        raw_id = det.id
        if isinstance(raw_id, (list, tuple)):
            if not raw_id:
                return
            self.latest_tag_id = int(raw_id[0])
        else:
            self.latest_tag_id = int(raw_id)

        if msg.header.frame_id:
            self.latest_frame_id = msg.header.frame_id

    # Periodically called function
    def _report_distance(self) -> None:
        if self.latest_tag_id is None:
            return

        camera_frames = [self.camera_frame]
        if self.latest_frame_id and self.latest_frame_id not in camera_frames:
            camera_frames.insert(0, self.latest_frame_id)

        tag_frames = [
            f"{self.tag_family}:{self.latest_tag_id}",
            f"tag_{self.latest_tag_id}",
            f"tag{self.latest_tag_id}",
            str(self.latest_tag_id),
        ]

        # Loop through frames
        for cam_frame in camera_frames:
            for tag_frame in tag_frames:
                try:
                    # Get transformed data
                    ts = self.tf_buffer.lookup_transform(
                        cam_frame,  # Frame we need
                        tag_frame,  # Source frame
                        Time(),     # Time we want the transform from (0 is latest)
                        timeout=Duration(seconds=self.lookup_timeout_sec),
                    )
                except Exception:
                    continue

                # Get data from the cam_frame
                x = float(ts.transform.translation.x)
                y = float(ts.transform.translation.y)
                z = float(ts.transform.translation.z)
                distance_3d = math.sqrt(x * x + y * y + z * z)
                distance_xy = math.sqrt(x * x + y * y)
                bearing_deg = math.degrees(math.atan2(y, x))
                # Optical frame convention (REP-103):
                # x = right, y = down, z = forward.
                depth_m = z
                height_up_m = -y

                print(
                    f"Tag {self.latest_tag_id} ({tag_frame}) from {cam_frame}: "
                    f"range3d={distance_3d:.2f}m, range_xy={distance_xy:.2f}m, "
                    f"bearing={bearing_deg:.1f}deg, depth(z)={depth_m:.2f}m, "
                    f"height_up(-y)={height_up_m:.2f}m"
                )
                return


def main() -> None:
    rclpy.init()
    node = TagDistanceReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
