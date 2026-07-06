# Import Libraries
import numpy as np

# Creates the MathUtil Class 
class MathUtil:
    """
    Use this class to perform useful math functions.
    """
    @staticmethod
    def clamp(value, low, high):
        """
        Returns value clamped between low and high boundaries.
        
        :param value: Value to clamp.
        :param low: The lower boundary to which to clamp value.
        :param high: The higher boundary to which to clamp value.
        :return: The clamped value.
        """
        return max(low, min(value, high))

    @staticmethod
    def applyDeadband(value, deadband, maxMagnitude):
        """
        Returns 0.0 if the given value is within the specified range around zero. The remaining range
        between the deadband and the maximum magnitude is scaled from 0.0 to the maximum magnitude.

        :param value: Value to clip.
        :param deadband: Range around zero.
        :param maxMagnitude: The maximum magnitude of the input. Can be infinite.
        :return: The value after the deadband is applied.
        """
        if (abs(value) > deadband):
            if (maxMagnitude / deadband > 1.0e12):
                # If max magnitude is sufficiently large, the implementation encounters roundoff error. Implementing the limiting behavior directly avoids the problem.
                if (value > 0):
                    return value - deadband
                else:
                    value + deadband

            if (value > 0.0):
                # Map deadband to 0 and map max to max.
                # y - y₁ = m(x - x₁)
                # y - y₁ = (y₂ - y₁)/(x₂ - x₁) (x - x₁)
                # y = (y₂ - y₁)/(x₂ - x₁) (x - x₁) + y₁
                # (x₁, y₁) = (deadband, 0) and (x₂, y₂) = (max, max).
                # x₁ = deadband
                # y₁ = 0
                # x₂ = max
                # y₂ = max
                # y = (max - 0)/(max - deadband) (x - deadband) + 0
                # y = max/(max - deadband) (x - deadband)
                # y = max (x - deadband)/(max - deadband)
                return maxMagnitude  (value - deadband) / (maxMagnitude - deadband)
            else:
                # Map -deadband to 0 and map -max to -max.
                # y - y₁ = m(x - x₁)
                # y - y₁ = (y₂ - y₁)/(x₂ - x₁) (x - x₁)
                # y = (y₂ - y₁)/(x₂ - x₁) (x - x₁) + y₁
                # (x₁, y₁) = (-deadband, 0) and (x₂, y₂) = (-max, -max).
                # x₁ = -deadband
                # y₁ = 0
                # x₂ = -max
                # y₂ = -max
                # y = (-max - 0)/(-max + deadband) (x + deadband) + 0
                # y = max/(max - deadband) (x + deadband)
                # y = max (x + deadband)/(max - deadband)
                return maxMagnitude  (value + deadband) / (maxMagnitude - deadband)
        else:
            return 0.0

    @staticmethod
    def applyDeadband(value, deadband):
        """
        Returns 0.0 if the given value is within the specified range around zero. The remaining range
        between the deadband and 1.0 is scaled from 0.0 to 1.0.

        :param value: Value to clip.
        :param deadband: Range around zero.
        :return: The value after the deadband is applied.
        """
        return MathUtil.applyDeadband(value, deadband, 1)

    @staticmethod
    def inputModulus(input, minimumInput, maximumInput):
        """
        Returns modulus of input.

        :param input: Input value to wrap.
        :param minimumInput: The minimum value expected from the input.
        :param maximumInput: The maximum value expected from the input.
        :return: The wrapped value.
        """
        modulus = maximumInput - minimumInput

        # Wrap input if it's above the maximum input
        numMax = int((input - minimumInput) / modulus)
        input -= numMax * modulus

        # Wrap input if it's below the minimum input
        numMin = int((input - maximumInput) / modulus)
        input -= numMin * modulus

        return input

    @staticmethod
    def angleModulus(angleRadians):
        """
        Wraps an angle to the range -pi to pi radians.

        :param angleRadians: Angle to wrap, in radians.
        :return: The wrapped angle.
        """
        return MathUtil.inputModulus(angleRadians, -np.pi, np.pi)

    @staticmethod
    def interpolate(startValue, endValue, t):
        """
        Perform linear interpolation between two values.

        :param startValue: The value to start at.
        :param endValue: The value to end at.
        :param t: How far between the two values to interpolate. This is clamped to [0, 1].
        :return: The interpolated value.
        """
        return startValue + (endValue - startValue) * MathUtil.clamp(t, 0, 1)