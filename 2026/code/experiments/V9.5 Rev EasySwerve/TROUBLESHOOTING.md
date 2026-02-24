# V9.5 Rev EasySwerve - Troubleshooting Log (Revised)
This ./python code was ported from java code in the ../java folder. it is for a rev easy swerve wheels with thru bore encoders v2 on each wheel

## Current Status (Autotune Implementation)

**What Works:**
- ✓ Rear left turn motor responds to PWM commands
- ✓ Motor reaches 0 radians without violent oscillation
- ✓ Ziegler-Nichols autotune implemented and callable via START+BACK buttons

**Current Problems:**
- ❌ Autotune returns kP=0.1 (returns default value, not calculated from relay test)
- ❌ Motor does NOT oscillate during autotune relay test (8 seconds of silence)
- ❌ Suspected root cause: Hard limits in hardware client still blocking motor during autotune
- ⚠️ Manual kP=0.1 works but motor doesn't auto-center on robot enable (no motion detected)


﻿﻿﻿﻿﻿﻿﻿ [ROBOT] === ENTERING TELEOP MODE === ﻿
﻿﻿﻿﻿﻿﻿ [ROBOT] Press START + BACK to begin Rear Left turn motor autotune ﻿
﻿﻿﻿﻿﻿﻿ [ROBOT] Teleop init complete ﻿
﻿﻿﻿﻿﻿﻿  ﻿
﻿﻿﻿﻿﻿﻿ [Rear Left] START: pos=  3.102 tgt= -3.142 kp=0.100000 pwm= 0.030 applied= 0.000 ﻿
﻿﻿﻿﻿﻿﻿ [Rear Left] pos=  3.102 err=  0.040 kp=0.100000 pwm= 0.030 applied= 0.000 ﻿
﻿﻿﻿﻿﻿﻿ [Rear Left] pos=  5.641 err=  0.642 kp=0.100000 pwm= 0.064 applied=-0.310 ﻿
﻿﻿﻿﻿﻿﻿ [Rear Left] pos=  8.334 err=  1.091 kp=0.100000 pwm= 0.109 applied= 0.030 ﻿
﻿﻿﻿﻿﻿﻿ [Rear Left] pos=  5.826 err=  0.457 kp=0.100000 pwm= 0.046 applied= 0.064 ﻿
﻿﻿﻿﻿﻿﻿ [Rear Left] pos=  6.173 err=  0.110 kp=0.100000 pwm= 0.030 applied= 0.109 ﻿
﻿﻿﻿﻿﻿﻿ [Rear Left] pos=  6.850 err= -0.567 kp=0.100000 pwm=-0.057 applied= 0.046 ﻿
﻿﻿﻿﻿﻿﻿ [Rear Left] pos=  6.378 err= -0.094 kp=0.100000 pwm=-0.030 applied= 0.030 ﻿
﻿﻿﻿﻿﻿﻿ [Rear Left] pos=  9.405 err=  0.020 kp=0.100000 pwm= 0.030 applied=-0.057 ﻿
﻿﻿﻿﻿﻿﻿ [Rear Left] pos=  6.795 err= -0.512 kp=0.100000 pwm=-0.051 applied=-0.030 ﻿
﻿﻿﻿﻿﻿﻿ [Rear Left] pos=  5.393 err=  0.890 kp=0.100000 pwm= 0.089 applied=-0.051 ﻿
﻿﻿﻿﻿﻿﻿ [Rear Left] pos=  5.932 err=  0.351 kp=0.100000 pwm= 0.035 applied=-0.051 ﻿
﻿﻿﻿﻿﻿﻿ [Rear Left] pos=  5.848 err=  0.436 kp=0.100000 pwm= 0.044 applied= 0.089 ﻿
﻿﻿﻿﻿﻿﻿ [Rear Left] pos=  6.466 err= -0.183 kp=0.100000 pwm=-0.030 applied= 0.035 ﻿
﻿﻿﻿﻿﻿﻿ [Rear Left] pos=  6.855 err= -0.572 kp=0.100000 pwm=-0.057 applied=-0.030 ﻿
﻿﻿﻿﻿﻿﻿ [Rear Left] pos=  5.662 err=  0.621 kp=0.100000 pwm= 0.062 applied=-0.057 ﻿
﻿﻿﻿﻿﻿﻿ [Rear Left] pos=  6.185 err=  0.098 kp=0.100000 pwm= 0.030 applied=-0.057 ﻿
﻿﻿﻿﻿﻿﻿ [Rear Left] pos=  6.060 err=  0.223 kp=0.100000 pwm= 0.030 applied= 0.062 ﻿
﻿﻿﻿﻿﻿﻿ [Rear Left] pos=  5.948 err=  0.335 kp=0.100000 pwm= 0.034 applied= 0.030 ﻿
﻿﻿﻿﻿﻿﻿ [Rear Left] pos=  6.200 err=  0.084 kp=0.100000 pwm= 0.030 applied= 0.030 ﻿
﻿﻿﻿﻿﻿﻿ [Rear Left] pos=  7.003 err= -0.720 kp=0.100000 pwm=-0.072 applied= 0.034 ﻿
﻿﻿﻿﻿﻿﻿ [Rear Left] pos=  3.267 err= -0.126 kp=0.100000 pwm=-0.030 applied= 0.030 ﻿
﻿﻿﻿﻿﻿﻿ [Rear Left] pos=  4.717 err=  1.566 kp=0.100000 pwm= 0.157 applied=-0.072 ﻿
﻿﻿﻿﻿﻿﻿ [Rear Left] pos=  6.038 err=  0.245 kp=0.100000 pwm= 0.030 applied=-0.030 ﻿
﻿﻿﻿﻿﻿﻿ [Rear Left] pos=  5.968 err=  0.315 kp=0.100000 pwm= 0.032 applied= 0.157 ﻿
﻿﻿﻿﻿﻿﻿ [Rear Left] pos=  3.241 err= -0.100 kp=0.100000 pwm=-0.030 applied= 0.030 ﻿
﻿﻿﻿﻿﻿﻿ [ROBOT] === EXITING TELEOP MODE === ﻿


**Autotune Usage:**
1. Press START + BACK during teleop to trigger 8-second relay tuning
2. Motor should oscillate left/right repeatedly during test  
3. Results print to console with calculated kP, kI, kD values
4. Copy values to `constants.py` TurnMotorGains class
5. Redeploy - gains load automatically on startup

**Next Steps:**
- Verify motor **is actually moving** during autotune (currently silent = not moving)
- Check hardware client hard limit settings for all turn motors
- May need `kNoPersistParameters` in SparkMaxConfig to respect hardware client settings

---

**1. minimize to rear left wheel**
- the code has been reduced down to focus on the rear left turning motors
- when the bot is enabled the wheel violently rotates left and right
- the goal is for the wheel to center on 0
this is sample data output from the last test:

 