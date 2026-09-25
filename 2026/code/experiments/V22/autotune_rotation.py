#!/usr/bin/env python3
"""
Autotune script for rotation PID controller.

Run this during robot testing to automatically tune the heading (rotation) PID gains.
The robot will perform controlled oscillations to measure system response characteristics.

Usage:
	python autotune_rotation.py
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from waypoint_navigator import WaypointNavigator
from swerve.swerve_drive import SwerveDrive


def autotune_interactive():
	"""Interactive autotune session"""
	print("\n" + "="*60)
	print("ROTATION PID AUTOTUNE - Interactive Mode")
	print("="*60)
	
	try:
		# Initialize hardware
		print("\n[INIT] Creating SwerveDrive...")
		drive = SwerveDrive()
		print("[INIT] Creating WaypointNavigator...")
		navigator = WaypointNavigator(drive)
		
		print("\n[READY] Hardware initialized and ready for autotune")
		print("\nAutotune parameters:")
		print("  - Target heading: 45°")
		print("  - Max power: 0.5 (safety limited)")
		print("  - Duration: 15 seconds max")
		print("  - Cycles: 3 oscillations")
		print("\n⚠️  IMPORTANT: Robot must be ELEVATED (on chucks, off the ground)")
		print("    Ground friction will distort measurements!")
		
		input("\nPress ENTER to START autotune (robot will rotate)...")
		
		# Run autotune
		result = navigator.autotune_rotation(
			target_heading=45.0,
			max_power=0.5,
			duration_seconds=15.0
		)
		
		print("\n" + "="*60)
		print("AUTOTUNE RESULTS")
		print("="*60)
		
		if result['success']:
			print(f"\n[OK] SUCCESS - New PID gains calculated:")
			print(f"  kP (proportional): {result['kp']:.8f}")
			print(f"  kI (integral):     {result['ki']:.8f}")
			print(f"  kD (derivative):   {result['kd']:.8f}")
			print(f"\nMeasurements:")
			print(f"  Ultimate period:   {result['period']:.4f} seconds")
			print(f"  Amplitude:         {result['amplitude']:.2f} degrees")
			print(f"  Oscillation cycles: {result['cycles']}")
			print(f"\nTo use these gains, update waypoint_navigator.py:")
			print(f"  self.pid_rotate = PIDController(kp={result['kp']:.8f}, ki={result['ki']:.8f}, kd={result['kd']:.8f}, name=\"Nav_Rotate\")")
		else:
			print(f"\n[FAIL] FAILED: {result['message']}")
			print("Troubleshooting:")
			print("  - Check that robot can rotate freely")
			print("  - Increase max_power if system is over-damped")
			print("  - Check gyro/heading sensor calibration")
		
		print("\n" + "="*60)
		drive.stop_all()
		
	except Exception as e:
		print(f"\n[ERROR] {type(e).__name__}: {str(e)}")
		import traceback
		traceback.print_exc()
		return False
	
	return result.get('success', False)


if __name__ == "__main__":
	try:
		success = autotune_interactive()
		sys.exit(0 if success else 1)
	except KeyboardInterrupt:
		print("\n[INTERRUPTED] Autotune cancelled by user")
		sys.exit(1)
