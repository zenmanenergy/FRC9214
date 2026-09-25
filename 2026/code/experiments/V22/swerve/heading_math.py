"""Shared heading/angle-wrapping helpers used by the navigator and drive code."""


def shortest_angle_diff(a: float, b: float) -> float:
	"""Signed shortest difference (b - a) wrapped to [-180, 180] degrees."""
	diff = (b - a) % 360.0
	if diff > 180.0:
		diff -= 360.0
	return diff


def lerp_angle(a: float, b: float, t: float) -> float:
	"""Shortest-path interpolation from heading a to heading b, wrapped to [0, 360)."""
	return (a + shortest_angle_diff(a, b) * t) % 360.0
