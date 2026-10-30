# 21: More buttons

## What it does

Connect several controller buttons to commands.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Plug an Xbox controller into the Driver Station laptop (it should be in slot 0 on the USB tab), clear some space, and enable in **Teleoperated**. Press A, B, X, Y and Start while driving.

## The code to look at

```python
# A: stop the wheels
self.controller.a().onTrue(self.drivetrain.stop())

# X (hold): lock the wheels
self.controller.x().whileTrue(self.drivetrain.lockWheels())
```

- **onTrue** starts the command once when the button is pressed. It keeps running until it finishes, even if you let go.
- **whileTrue** runs the command while the button is held and stops it when you let go.
- **Y and B use turnTo**, so pressing them turns the robot to face a direction.

## Key ideas

- **Only one command uses the drivetrain at a time.** Pressing X interrupts driving with the sticks. When the X command ends, stick driving comes back automatically.
- **Pick onTrue or whileTrue by what the command does.** Something that should happen once (turn to face): onTrue. Something that should last as long as you hold (lock): whileTrue.

## Try this

1. Change X from whileTrue to toggleOnTrue: press once to lock, press again to unlock.
2. Give the left bumper turnTo(90).
3. Press Y, then push the stick before it finishes turning. What happens? (Nothing: the stick only works when no command is using the drivetrain.)

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
