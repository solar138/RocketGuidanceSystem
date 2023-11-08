# RocketGuidanceSystem
Works for ATMega328-based avionics systems, specifically Arduino Nano. Controls 4 fins to guide a rocket to a target altitude. Originally designed for the TARC competition.

## Features
* Controls 4 mid-fins with servos
* Individual fin calibration
* Rocket position logging
* Altitude undershoot compensation

## Parameters
  ### Data Logging:
* LOG_LENGTH: Length of the data log in # of entries
* LOG_INTERVAL: Number of seconds between log entries
  ### Pins
* X_PIN: Set to the appropriate pins that the servo control signals are connected to. (Must be digital pins that have PWM)
* The accel/gyro I2C pins must be connected to pins A4 (SDA) and A5 (SCL). This is default for Arduino Nano boards but may be different for other boards.
  ### Math Constants
* gravity (m/s^2): Gravitational acceleration. Must be negative. (Default: -9.81 m/s^2)
  ### Guidance Constants
* targetHeight (m): Target altitude to guide to. (Default: 250 m)
* launchAccelThreshold (m/s^2): Acceleration needed to consider the launch started.
* velocityOvershootFactor: Offset value added to the delta height based on velocity.
  ### Per-Fin Calibrations
* neutralAngleX (degrees): Neutral angle (0 degree point) for each fin.
* angleMultiplierX (degrees/m): Multiplier of delta height to angle.
* maxAngle (degrees): Maximum rotate angle of each fin.

## Rocket Construction
There should be four mid-fins each placed at right-angles from eachother. Each fin should be connected to a standard servo with the min and max limits being when the fins are perpendicular to the rocket body. The middle range of each servo should be when the fins are parallel with the body. Fine tuning may be adjusted in software but it is easier to mount the fins in the correct direction from the start. Generally tail fins are also required for stability so the CP (Center of Pressure) stays below the CM/CG (Center of Mass/Gravity).

![image](https://github.com/solar138/RocketGuidanceSystem/assets/46548002/376c6479-3522-4ece-b4a3-b7f53f94850f)

Figure 1: Sample CAD model of a rocket design.
