# 1. FRC and control-system foundations

[Guide index](README.md) · [Next: program flow](02-program-flow.md)

This guide assumes familiarity with Java, including classes, inheritance, collections, lambdas, and method references. This chapter introduces the robotics concepts needed to read the implementation.

## The runtime and hardware boundary

The roboRIO runs the Java application. WPILib provides the robot lifecycle, Driver Station interface, command scheduler, geometry, kinematics, and estimation libraries. Vendor libraries expose specific devices: CTRE Phoenix for motors and sensors, PhotonVision for camera results, and PathPlanner for autonomous trajectories.

Constructing a subsystem can communicate with real hardware. For example, `new IntakeSubsystem()` creates motor-controller objects, zeros the arm's reported position, and applies configurations. Constructors in this repository are not dependency-free domain objects; instantiating `RobotContainer` creates most of the robot's hardware interface.

The CAN bus connects addressable devices. A CAN ID is an address on a particular bus. `new TalonFX(id, kCANBus)` selects an explicit bus; this project names it `CANivore`. Constructors without that argument use their default bus. The source bus and device ID must match the physical configuration.

Motor controllers execute their own control logic between Java updates. A position or velocity request can remain active after the calling method returns. A command ending does not inherently stop a motor; its cleanup must send the intended final request. Disabled state also suppresses physical motor operation through the robot control system.

## The periodic execution model

`TimedRobot` normally runs a 20 ms loop, about 50 Hz. This application calls `CommandScheduler.run()` from `robotPeriodic()`. Most application logic executes incrementally within that loop:

1. Read current measurements or input.
2. Calculate an output or update state.
3. Return promptly so other subsystems and commands can run.

Do not block a periodic callback while waiting for motion to complete. Model the action as a command that checks progress across scheduler cycles. The autonomous library and motor controllers may have their own internal behavior, but the application's periodic methods are not independent background threads.

A subsystem groups hardware and operations. A command represents an action over time. Command requirements provide scheduler-level ownership; they are not locks on Java methods and do not prevent unrelated code from directly calling the same motor setter. See [program flow](02-program-flow.md) for the lifecycle and ownership rules used here.

## Measurements and physical references

The drivetrain combines several measurements:

- Drive motor encoders report shaft position and velocity, converted to wheel travel through gearing and wheel radius.
- CANcoders report absolute steering angles within a rotation.
- The Pigeon2 gyro reports yaw, or orientation about the vertical axis.
- PhotonVision reports camera observations that can produce a field pose estimate.

An absolute encoder can identify an angle within a rotation without counting from a startup zero. It still needs the correct physical mounting offset and direction. Setting a motor's reported position to zero establishes a software reference; it does not physically home a mechanism.

A **pose** is position plus orientation. `Pose2d` represents planar x/y and heading. A **transform** expresses translation and rotation between frames, such as the camera mount relative to the robot. A **coordinate frame** specifies the origin and axes used to interpret a vector or pose. Robot-relative and field-relative motion are different representations of the same physical motion.

## Units and conventions

Most hardware API values are plain doubles, so dimensional correctness is the caller's responsibility.

| Quantity | Representation used here |
| --- | --- |
| Field distance | Meters |
| Chassis and wheel speed | Meters per second |
| Chassis angular speed | Radians per second |
| Raw TalonFX positions | Rotations, subject to configured feedback ratios |
| Raw TalonFX velocity | Rotations per second, subject to configured feedback ratios |
| Mechanism RPM APIs | Revolutions per minute; methods divide or multiply by 60 |
| `Rotation2d` | Angle with explicit degree, radian, and rotation conversions |
| Open-loop motor output | Output fraction, normally between −1 and +1 |
| Motor voltage request | Volts |

One rotation equals 360° or `2π` radians. A ratio configured on a motor controller can change whether reported rotations refer to the rotor, sensor, or mechanism. Inspect feedback configuration before adding another conversion in Java.

WPILib's planar convention uses +x forward, +y left, and positive rotation counterclockwise. Field coordinates use fixed field axes; robot coordinates turn with the robot. The detailed conversions and the distinction between raw gyro heading and estimated field heading are covered in [swerve](03-swerve-drive.md).

## Feedback and feedforward

A closed-loop controller compares a **setpoint** with a measurement and uses their error to choose output. For example, requested wheel speed 2 m/s and measured speed 1.5 m/s produce an error of 0.5 m/s.

PID control combines:

- **Proportional** response to current error, scaled by `kP`.
- **Integral** response to accumulated error, scaled by `kI`.
- **Derivative** response to changing error, scaled by `kD`.

Gains have units determined by the controlled quantity and output. A wheel-speed-to-voltage gain cannot be substituted for a field-position-to-chassis-speed gain. This project has separate wheel propulsion, steering, heading, and autonomous translation controllers.

**Feedforward** estimates the output needed for requested motion without waiting for measured error. The drive code adds feedforward voltage to PID feedback. Its static term helps overcome friction; its velocity term estimates voltage needed to sustain speed. The configured acceleration term is zero.

**Open loop** directly requests output without regulating measured speed or position. `motor.set(0.4)` requests 40% output, not a guaranteed RPM. Closed-loop control may run on the roboRIO or inside a motor controller. For example, `moveIntake()` implements a simple position loop in Java by selecting −0.2, 0, or +0.2 output based on measured position; steering uses the TalonFX's internal position control.

A **tolerance** defines acceptable error. The intake's tolerance avoids continually reversing output around an exact target. More elaborate controllers may also need velocity tolerances, acceleration limits, or settling-time checks; those should not be assumed unless implemented.

## State estimation and uncertainty

Odometry integrates wheel travel and gyro observations to estimate pose. It drifts because of slip, calibration error, and sensor noise. Vision supplies an independent field reference but has latency and occasional outliers.

The pose estimator combines these sources using uncertainty settings. A standard deviation expresses how uncertain a measurement is; a smaller value gives that observation greater influence. A timestamp lets the estimator relate delayed vision to the time of observation rather than treating it as a current measurement. Filtering and covariance choices are part of behavior, not merely telemetry settings.

See [vision and autonomous](05-vision-and-autonomous.md) for the actual filters, uncertainty calculation, and known limitations in this implementation.
