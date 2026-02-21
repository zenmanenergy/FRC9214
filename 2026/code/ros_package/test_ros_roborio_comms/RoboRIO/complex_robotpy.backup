#!/usr/bin/env python3

"""
Summary:

- Mode bit (test_mode_enabled) toggled from NT/ROS
- Profile selector (test_profile: "sine" or "square")
- Square-path generator (forward + rotate, repeats)
- Noise injection (configurable, Gaussian)
- Timestamps (FPGA seconds + boot-relative seconds) published to NT
- Deadman/stale protection for external commands (cmd_vel_stamp_s + timeout)
- And a ROS 2 shoreside test node that:
- Subscribes to /cmd_vel + /odom
- Live-plots time series + XY path (matplotlib)
- Prints sanity checks + can optionally log CSV

--------------------------------------------------------------------------

Flags:

    When USE_INTERNAL_TEST_CMDS = True:

        1. vx ramps 0 → +1.0 m/s → −1.0 m/s (triangle wave)
        2. vy is a slow sine wave
        3. wz is a constant rotation rate (easy to see yaw accumulation)

    Odometry

        1. Publishes non-zero odom_* continuously
        2. Makes it obvious if frames or units are wrong
        3. When DEFAULT_SIM_ODOM_ENABLED = True
            publishes generated odom even if you have no sensors
        3. When DEFAULT_SIM_ODOM_MODE =
            "integrate_cmd" (uses cmd integration) 
            "pattern" (independent motion pattern)
        4. DEFAULT_SIM_ODOM_USE_CMD_SOURCE =
            True --> uses the same cmd source you selected (test/external)
            False --> uses its own pattern
        5. odom_source string key so shoreside can see what it’s getting

        This is designed so you can run external commands from ROS, 
        but still have changing odom without hardware.

    What you should see from ROS2 shoreside

        If everything is wired correctly:

            cmd_vel_* changes even without ROS publishing
            odom_x/y trace curves, not straight lines
            odom_yaw increases steadily

            Heartbeat increments continuously

        This makes it very obvious whether:

            NT is reachable
            Keys are named correctly
            Units/signs match expectations

        Your ROS2 bridge is functional

How to use it (typical workflows)

    A) No hardware, want obvious motion without ROS

        test_mode_enabled = true
        test_profile = "square"
        sim_odom_enabled = true
        sim_odom_mode = "integrate_cmd"
        noise_enable = false (initially)

        You’ll see square-ish XY path + yaw steps.

    B) Want to test ROS commanding but still no sensors

        test_mode_enabled = false (ROS drives cmd_vel_*)
        sim_odom_enabled = true
        sim_odom_mode = "integrate_cmd" (fake odom follows ROS commands)

    C) Want odom changing even if ROS is idle / zero

        sim_odom_enabled = true
        sim_odom_mode = "pattern"
        sim_odom_use_cmd_source = false

        This forces a motion pattern regardless of cmd_vel.

    NT keys to watch shoreside

        odom_source (string): "sim_integrate_cmd", "sim_pattern", "hardware_placeholder"

        active_cmd_source (string): "test:square", "test:sine", "external" / "external_stale(...)"

        robot_fpga_time_s, robot_uptime_s, robot_heartbeat
"""

import math
import random
import wpilib
from wpilib import Timer


class MyRobot(wpilib.TimedRobot):
    # ============================
    # DEFAULTS
    # ============================
    DEFAULT_TEST_MODE = True
    DEFAULT_PROFILE = "square"  # "sine" or "square"

    # Test cmd params
    VX_MAX = 1.0
    VY_AMP = 0.5
    WZ_TEST = 0.3

    SQ_FWD_VX = 0.8
    SQ_LEG_TIME = 2.5
    SQ_TURN_WZ = 0.9
    SQ_TURN_TIME = (math.pi / 2.0) / SQ_TURN_WZ  # ~90deg

    # External command safety
    CMD_TIMEOUT_S = 0.25

    # Noise on published odom
    DEFAULT_NOISE_ENABLE = False
    DEFAULT_NOISE_ODOM_POS_STD_M = 0.02
    DEFAULT_NOISE_ODOM_YAW_STD_RAD = 0.01
    DEFAULT_NOISE_ODOM_VEL_STD_MPS = 0.02
    DEFAULT_NOISE_ODOM_WZ_STD_RADPS = 0.01

    # ============================
    # SIM ODOM SETTINGS (NEW)
    # ============================
    DEFAULT_SIM_ODOM_ENABLED = True
    DEFAULT_SIM_ODOM_MODE = "integrate_cmd"   # "integrate_cmd" or "pattern"
    DEFAULT_SIM_ODOM_USE_CMD_SOURCE = True    # if False, pattern ignores cmds
    DEFAULT_SIM_ODOM_PATTERN_SPEED = 0.6      # used in pattern mode

    def robotInit(self):
        self.sd = wpilib.SmartDashboard

        # command inputs (ROS -> NT -> robot)
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0

        # internal time
        self.boot_t0 = Timer.getFPGATimestamp()
        self.last_t = self.boot_t0
        self.heartbeat = 0

        # "truth" internal state (used for sim odom integrate_cmd)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # square cmd FSM
        self.sq_state = "FWD"
        self.sq_state_t = 0.0

        # sim-odom pattern phase
        self.pattern_phase = 0.0

        # ----------------------------
        # Seed keys
        # ----------------------------
        self.sd.putNumber("robot_heartbeat", 0.0)

        # cmd test toggles
        self.sd.putBoolean("test_mode_enabled", bool(self.DEFAULT_TEST_MODE))
        self.sd.putString("test_profile", str(self.DEFAULT_PROFILE))  # "sine"/"square"
        self.sd.putString("active_cmd_source", "unknown")             # "test:*" or "external*"

        # external cmd keys
        self.sd.putNumber("cmd_vel_vx_mps", 0.0)
        self.sd.putNumber("cmd_vel_vy_mps", 0.0)
        self.sd.putNumber("cmd_vel_wz_radps", 0.0)
        self.sd.putNumber("cmd_vel_stamp_s", 0.0)

        # timestamps
        self.sd.putNumber("robot_fpga_time_s", 0.0)
        self.sd.putNumber("robot_uptime_s", 0.0)

        # noise controls
        self.sd.putBoolean("noise_enable", bool(self.DEFAULT_NOISE_ENABLE))
        self.sd.putNumber("noise_odom_pos_std_m", float(self.DEFAULT_NOISE_ODOM_POS_STD_M))
        self.sd.putNumber("noise_odom_yaw_std_rad", float(self.DEFAULT_NOISE_ODOM_YAW_STD_RAD))
        self.sd.putNumber("noise_odom_vel_std_mps", float(self.DEFAULT_NOISE_ODOM_VEL_STD_MPS))
        self.sd.putNumber("noise_odom_wz_std_radps", float(self.DEFAULT_NOISE_ODOM_WZ_STD_RADPS))

        # ----------------------------
        # SIM ODOM CONTROLS (NEW)
        # ----------------------------
        self.sd.putBoolean("sim_odom_enabled", bool(self.DEFAULT_SIM_ODOM_ENABLED))
        self.sd.putString("sim_odom_mode", str(self.DEFAULT_SIM_ODOM_MODE))  # integrate_cmd|pattern
        self.sd.putBoolean("sim_odom_use_cmd_source", bool(self.DEFAULT_SIM_ODOM_USE_CMD_SOURCE))
        self.sd.putNumber("sim_odom_pattern_speed_mps", float(self.DEFAULT_SIM_ODOM_PATTERN_SPEED))
        self.sd.putString("odom_source", "unknown")  # sim_integrate_cmd|sim_pattern|hardware_placeholder

        # odom keys
        for k in (
            "odom_x_m", "odom_y_m", "odom_yaw_rad",
            "odom_vx_mps", "odom_vy_mps", "odom_wz_radps",
        ):
            self.sd.putNumber(k, 0.0)

    def robotPeriodic(self):
        now = Timer.getFPGATimestamp()
        self.heartbeat += 1
        self.sd.putNumber("robot_heartbeat", float(self.heartbeat))
        self.sd.putNumber("robot_fpga_time_s", float(now))
        self.sd.putNumber("robot_uptime_s", float(now - self.boot_t0))

    # ============================
    # COMMAND GENERATORS
    # ============================
    def generate_test_commands_sine(self, t):
        period = 8.0
        phase = (t % period) / period
        if phase < 0.5:
            vx = 2.0 * phase * self.VX_MAX
        else:
            vx = (2.0 - 2.0 * phase) * self.VX_MAX

        vy = self.VY_AMP * math.sin(2.0 * math.pi * 0.25 * t)
        wz = self.WZ_TEST
        return vx, vy, wz

    def generate_test_commands_square(self, dt):
        self.sq_state_t += dt

        if self.sq_state == "FWD":
            vx, vy, wz = self.SQ_FWD_VX, 0.0, 0.0
            if self.sq_state_t >= self.SQ_LEG_TIME:
                self.sq_state = "TURN"
                self.sq_state_t = 0.0
        else:  # TURN
            vx, vy, wz = 0.0, 0.0, self.SQ_TURN_WZ
            if self.sq_state_t >= self.SQ_TURN_TIME:
                self.sq_state = "FWD"
                self.sq_state_t = 0.0

        return vx, vy, wz

    def read_test_controls(self):
        test_mode = bool(self.sd.getBoolean("test_mode_enabled", self.DEFAULT_TEST_MODE))
        profile = str(self.sd.getString("test_profile", self.DEFAULT_PROFILE)).strip().lower()
        if profile not in ("sine", "square"):
            profile = "sine"
        return test_mode, profile

    def get_external_cmds_or_zero(self, now):
        stamp = float(self.sd.getNumber("cmd_vel_stamp_s", 0.0))
        age = now - stamp
        if stamp <= 0.0 or age > self.CMD_TIMEOUT_S:
            return 0.0, 0.0, 0.0, False, age

        vx = float(self.sd.getNumber("cmd_vel_vx_mps", 0.0))
        vy = float(self.sd.getNumber("cmd_vel_vy_mps", 0.0))
        wz = float(self.sd.getNumber("cmd_vel_wz_radps", 0.0))
        return vx, vy, wz, True, age

    # ============================
    # SIM ODOM 
    # ============================
    def read_sim_odom_controls(self):
        en = bool(self.sd.getBoolean("sim_odom_enabled", self.DEFAULT_SIM_ODOM_ENABLED))
        mode = str(self.sd.getString("sim_odom_mode", self.DEFAULT_SIM_ODOM_MODE)).strip().lower()
        if mode not in ("integrate_cmd", "pattern"):
            mode = "integrate_cmd"
        use_cmd_source = bool(self.sd.getBoolean("sim_odom_use_cmd_source", self.DEFAULT_SIM_ODOM_USE_CMD_SOURCE))
        spd = float(self.sd.getNumber("sim_odom_pattern_speed_mps", self.DEFAULT_SIM_ODOM_PATTERN_SPEED))
        return en, mode, use_cmd_source, spd

    def sim_odom_from_pattern(self, dt, speed_mps):
        """
        Generates changing odom independent of cmd_vel:
          - moves in a circle-ish path
          - yaw advances steadily
        """
        self.pattern_phase += dt

        # pattern velocities (body frame-ish for publishing)
        vx = speed_mps
        vy = 0.2 * speed_mps * math.sin(2.0 * math.pi * 0.2 * self.pattern_phase)
        wz = 0.4  # rad/s constant-ish

        # integrate like the normal model
        self.yaw += wz * dt
        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)
        vx_w = vx * cy - vy * sy
        vy_w = vx * sy + vy * cy
        self.x += vx_w * dt
        self.y += vy_w * dt

        return vx, vy, wz

    def apply_noise(self, x, y, yaw, vx, vy, wz):
        noise_en = bool(self.sd.getBoolean("noise_enable", self.DEFAULT_NOISE_ENABLE))
        if not noise_en:
            return x, y, yaw, vx, vy, wz

        pos_std = float(self.sd.getNumber("noise_odom_pos_std_m", self.DEFAULT_NOISE_ODOM_POS_STD_M))
        yaw_std = float(self.sd.getNumber("noise_odom_yaw_std_rad", self.DEFAULT_NOISE_ODOM_YAW_STD_RAD))
        vel_std = float(self.sd.getNumber("noise_odom_vel_std_mps", self.DEFAULT_NOISE_ODOM_VEL_STD_MPS))
        wz_std  = float(self.sd.getNumber("noise_odom_wz_std_radps", self.DEFAULT_NOISE_ODOM_WZ_STD_RADPS))

        return (
            x + random.gauss(0.0, pos_std),
            y + random.gauss(0.0, pos_std),
            yaw + random.gauss(0.0, yaw_std),
            vx + random.gauss(0.0, vel_std),
            vy + random.gauss(0.0, vel_std),
            wz + random.gauss(0.0, wz_std),
        )

    def publish_odom(self, x, y, yaw, vx, vy, wz, source):
        x, y, yaw, vx, vy, wz = self.apply_noise(x, y, yaw, vx, vy, wz)

        self.sd.putNumber("odom_x_m", float(x))
        self.sd.putNumber("odom_y_m", float(y))
        self.sd.putNumber("odom_yaw_rad", float(yaw))
        self.sd.putNumber("odom_vx_mps", float(vx))
        self.sd.putNumber("odom_vy_mps", float(vy))
        self.sd.putNumber("odom_wz_radps", float(wz))
        self.sd.putString("odom_source", source)

    def teleopPeriodic(self):
        now = Timer.getFPGATimestamp()
        dt = max(1e-3, now - self.last_t)
        self.last_t = now

        # ============================
        # COMMAND SOURCE
        # ============================
        test_mode, profile = self.read_test_controls()

        if test_mode:
            t = now - self.boot_t0
            if profile == "square":
                self.cmd_vx, self.cmd_vy, self.cmd_wz = self.generate_test_commands_square(dt)
            else:
                self.cmd_vx, self.cmd_vy, self.cmd_wz = self.generate_test_commands_sine(t)

            # publish commands + stamp (so ROS can see motion even in test)
            self.sd.putNumber("cmd_vel_vx_mps", float(self.cmd_vx))
            self.sd.putNumber("cmd_vel_vy_mps", float(self.cmd_vy))
            self.sd.putNumber("cmd_vel_wz_radps", float(self.cmd_wz))
            self.sd.putNumber("cmd_vel_stamp_s", float(now))

            self.sd.putString("active_cmd_source", f"test:{profile}")

        else:
            vx, vy, wz, ok, age = self.get_external_cmds_or_zero(now)
            self.cmd_vx, self.cmd_vy, self.cmd_wz = vx, vy, wz
            self.sd.putString("active_cmd_source", "external" if ok else f"external_stale(age={age:.3f}s)")

        # ============================
        # ODOM SOURCE (SIM vs HARDWARE)
        # ============================
        sim_en, sim_mode, sim_use_cmd, pat_speed = self.read_sim_odom_controls()

        if sim_en:
            if sim_mode == "pattern" and not sim_use_cmd:
                # completely independent odom pattern
                vx_o, vy_o, wz_o = self.sim_odom_from_pattern(dt, pat_speed)
                self.publish_odom(self.x, self.y, self.yaw, vx_o, vy_o, wz_o, "sim_pattern")

            else:
                # integrate the selected command source (test or external)
                # (This is the simplest "fake odom" when you have no sensors.)
                self.yaw += self.cmd_wz * dt
                cy = math.cos(self.yaw)
                sy = math.sin(self.yaw)
                vx_w = self.cmd_vx * cy - self.cmd_vy * sy
                vy_w = self.cmd_vx * sy + self.cmd_vy * cy
                self.x += vx_w * dt
                self.y += vy_w * dt

                self.publish_odom(self.x, self.y, self.yaw, self.cmd_vx, self.cmd_vy, self.cmd_wz, "sim_integrate_cmd")

        else:
            # Placeholder for hardware odom path:
            # You would read encoders/gyro here and publish them.
            # For now publish zeros but clearly label it.
            self.publish_odom(self.x, self.y, self.yaw, 0.0, 0.0, 0.0, "hardware_placeholder")

    def disabledInit(self):
        self.last_t = Timer.getFPGATimestamp()


if __name__ == "__main__":
    wpilib.run(MyRobot)