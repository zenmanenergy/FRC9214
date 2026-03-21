import math
import swerve_config as config

def drive_simple(self, pilot_joystick):
    """
    360-degree continuous rotation with permanent wheel offsets.
    Wheel angles are based entirely on right stick position.
    All wheels run at constant speed.

    Wheel offsets:
    - rear_left: -90°
    - rear_right: 0°
    - front_left: 90°
    - front_right: 180°

    Args:
        self: SwerveDrive instance
        pilot_joystick: PilotJoystick instance to read right stick inputs from
    """
    

    # Get right joystick inputs from pilot joystick
    x = pilot_joystick.get_right_x()

    # Deadzone
    if abs(x) < 0.0:
        self.stop_all()
        return

    # --- CONTINUOUS JOYSTICK ANGLE (0-360°) ---
    #joystick_angle = math.degrees(math.atan2(x, y)) % 360

    # --- PERMANENT WHEEL OFFSETS ---
    right_angles = {
        "rear_left": 45,      # bottom left
        "rear_right": 135,       # bottom right
        "front_left": 315,      # top left
        "front_right": 225     # top right
    }

    left_angles = {
        "rear_left": 225,      # bottom left
        "rear_right": 315,       # bottom right
        "front_left": 135,      # top left
        "front_right": 45    # top right
    }

    # --- CONSTANT SPEED ---
    drive_power = config.MOTOR_SCALE_TELEOP  # Full constant speed
    
    if x <= 0:
        angles = left_angles
    else:
        angles = right_angles
    # --- APPLY TO ALL WHEELS ---
    for wheel_name, angle in angles.items():
        if wheel_name in ["front_right", "rear_left", "front_left", "rear_right"]:
           
            # Invert drive power for these wheels to maintain consistent rotation direction
            drive_power = -config.MOTOR_SCALE_TELEOP
           # target_angle = (joystick_angle + offset) % 360
            self.drive_wheel_to_angle(wheel_name, angle)
            self.wheels[wheel_name].set_drive_power(drive_power)

    self.update_single_wheel_alignment()

