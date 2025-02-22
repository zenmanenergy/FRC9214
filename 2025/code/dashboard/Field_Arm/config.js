// Centralized configuration file for robot arm constraints
const LIMITS = {
	ELEVATOR_MIN: 0,
	ELEVATOR_MAX: 100,
	ARM_MIN: -60,
	ARM_MAX: 200,
	WRIST_MIN: -120,
	WRIST_MAX: 120,
	GRABBER_MIN: -100,
	GRABBER_MAX: 100
};

// Track grabber rotation speed
let grabberRotationSpeed = 0; // Default is no movement
let lastTimestamp = 0;

// Global mode variable (starts in drawing mode)
let mode = "drawing";

// Ensure it's accessible globally
window.mode = mode;
