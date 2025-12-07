import wpilib
from wpilib import RobotBase
import socket
import struct
import time

# One-time TCP connection
webots_socket = None
motor_outputs = {"left": 0.0, "right": 0.0}


def init_webots_socket():
    global webots_socket
    if webots_socket is not None:
        return  # Already connected
    webots_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    webots_socket.settimeout(2.0)
    while True:
        try:
            webots_socket.connect(('127.0.0.1', 9999))
            break
        except ConnectionRefusedError:
            print("Webots bridge not ready, retrying in 0.5s...")
            time.sleep(0.5)
        except Exception as e:
            print(f"Socket error: {e}")
            time.sleep(0.5)
    webots_socket.settimeout(None)
    print("Connected to Webots")

def send_to_webots():
    if webots_socket:
        data = struct.pack("ff", motor_outputs["left"], motor_outputs["right"])
        webots_socket.sendall(data)

IS_SIM = RobotBase.isSimulation()

# If not sim, use the real thing
if not IS_SIM:
    from phoenix5 import WPI_TalonSRX as RealTalon
    from phoenix5 import NeutralMode as RealNeutralMode
else:
    RealTalon = None
    RealNeutralMode = None


# Custom simulation version of NeutralMode enum
class NeutralMode:
    Coast = "Coast"
    Brake = "Brake"


class SimTalon:
    def __init__(self, device_id):
        self.device_id = device_id
        self.inverted = False
        self.neutral_mode = NeutralMode.Coast
        self.output = 0
        self.name = f"SimTalon_{device_id}"
        print(f"[SIM INIT] {self.name} created")

    def setInverted(self, is_inverted):
        self.inverted = is_inverted
        print(f"[SIM] {self.name}.setInverted({is_inverted})")

    def setNeutralMode(self, mode):
        self.neutral_mode = mode
        print(f"[SIM] {self.name}.setNeutralMode({mode})")

    def set(self, value):
        final_value = -value if self.inverted else value
        self.output = final_value
        print(f"[SIM] {self.name}.set({value}) → actual output: {final_value}")
        if self.device_id in [1, 2]:
            motor_outputs["left"] = final_value
        elif self.device_id in [3, 4]:
            motor_outputs["right"] = final_value
        send_to_webots()


# Wrapper that chooses real or sim on instantiation
class WPI_TalonSRX:
    def __new__(cls, device_id):
        if IS_SIM:
            return SimTalon(device_id)
        else:
            return RealTalon(device_id)
