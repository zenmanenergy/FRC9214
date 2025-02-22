// Centralized configuration file for robot arm constraints
const LIMITS = {
	ELEVATOR_MIN: 0,
	ELEVATOR_MAX: 100,
	ARM_MIN: -75,
	ARM_MAX: 200,
	WRIST_MIN: -150,
	WRIST_MAX: 150,
	GRABBER_MIN: -100,
	GRABBER_MAX: 100
};

// Scaling factor: 1 cm in real life = 5 pixels
const SCALE = 2;

// Define real-world sizes in centimeters, then convert to pixels
const ELEVATOR_MIN_HEIGHT = -18 * SCALE;
const ELEVATOR_MAX_HEIGHT = 70 * SCALE;

const ARM_LENGTH = 78 * SCALE;
const WRIST_LENGTH = 27.5 * SCALE;

const GRABBER_RADIUS = 5 * SCALE;
const TUSK_VERTICAL = 8 * SCALE;
const TUSK_HOOK = 19 * SCALE;
const spokeLength =5 * SCALE;

const BASE_WIDTH = 76 * SCALE;
const BASE_HEIGHT = 10 * SCALE;
const BASE_VERTICAL_SECTION_WIDTH = 12 * SCALE;
const BASE_VERTICAL_SECTION_HEIGHT = 98 * SCALE;

const WHEEL_RADIUS = 8 * SCALE;
const WHEEL_OFFSET = 25 * SCALE;
const CENTER_WHEEL_OFFSET = 0 * SCALE; // Middle wheel stays at the center

// Track grabber rotation speed
let grabberRotationSpeed = 0; // Default is no movement
let lastTimestamp = 0;

// Global mode variable (starts in drawing mode)
let mode = "drawing";

// Ensure it's accessible globally
window.mode = mode;

// Global variable for selected team (default to blue)
let selectedTeam = "blue"; // Can be "red" or "blue"

// Ensure it's accessible globally
window.selectedTeam = selectedTeam;

