import wpilib
from wpimath.geometry import Pose2d, Rotation2d


class VisionNetworkTables:
	"""Manages NetworkTables communication with vision/camera coprocessor."""
	
	def __init__(self, table_name: str = "vision"):
		"""
		Initialize vision network tables.
		
		Args:
			table_name: Name of the NetworkTables table to use
		"""
		self.nt = wpilib.networktables.NetworkTablesInstance.getDefault()
		self.nt.startClient4("robot")
		self.nt.setServerTeam(9214)
		
		# Create the vision table
		self.table = self.nt.getTable(table_name)
		
		# Subscribe to camera pose data
		self.x_sub = self.table.getDoubleTopic("x").subscribe(0.0)
		self.y_sub = self.table.getDoubleTopic("y").subscribe(0.0)
		self.heading_sub = self.table.getDoubleTopic("heading").subscribe(0.0)
		self.has_target_sub = self.table.getBooleanTopic("has_target").subscribe(False)
		self.timestamp_sub = self.table.getDoubleTopic("timestamp").subscribe(0.0)
		
		# Publish robot odometry for camera to see
		self.odom_x_pub = self.table.getDoubleTopic("robot/x").publish()
		self.odom_y_pub = self.table.getDoubleTopic("robot/y").publish()
		self.odom_heading_pub = self.table.getDoubleTopic("robot/heading").publish()
		self.odom_confidence_pub = self.table.getDoubleTopic("robot/confidence").publish()
		
		# Track last update time
		self.last_vision_update = 0.0
		self.vision_timeout = 1.0  # seconds
		
	def get_camera_pose(self) -> tuple[Pose2d, bool]:
		"""
		Get the latest camera-based pose.
		
		Returns:
			(Pose2d, has_valid_target) - pose from camera and whether it's valid
		"""
		has_target = self.has_target_sub.get()
		
		if not has_target:
			return Pose2d(), False
		
		x = self.x_sub.get()
		y = self.y_sub.get()
		heading = self.heading_sub.get()
		timestamp = self.timestamp_sub.get()
		
		# Check if data is stale
		current_time = wpilib.Timer.getFPGATimestamp()
		if current_time - timestamp > self.vision_timeout:
			return Pose2d(), False
		
		pose = Pose2d(x, y, Rotation2d.fromDegrees(heading))
		self.last_vision_update = current_time
		
		return pose, True
	
	def publish_odometry(self, pose: Pose2d, confidence: float = 0.95):
		"""
		Publish current odometry to NetworkTables for camera to see.
		
		Args:
			pose: Current robot pose from odometry
			confidence: Confidence in odometry (0-1)
		"""
		self.odom_x_pub.set(pose.X())
		self.odom_y_pub.set(pose.Y())
		self.odom_heading_pub.set(pose.rotation().degrees())
		self.odom_confidence_pub.set(confidence)
	
	def has_recent_vision_update(self) -> bool:
		"""Check if we have received a recent valid vision update."""
		current_time = wpilib.Timer.getFPGATimestamp()
		return (current_time - self.last_vision_update) < self.vision_timeout
