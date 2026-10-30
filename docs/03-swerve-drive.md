# 3. Swerve driving and heading control

[Guide index](README.md) · [Previous: program flow](02-program-flow.md) · [Next: mechanisms](04-mechanisms.md)

## Three layers of driving code

| Source | Responsibility |
| --- | --- |
| [TeleoperatedControlCommand.java](../src/main/java/frc/robot/commands/TeleoperatedControlCommand.java) | Read joystick axes and turn them into requested chassis speeds |
| [SwerveDriveSubsystem.java](../src/main/java/frc/robot/subsystems/SwerveDriveSubsystem.java) | Coordinate four modules, hold heading, track pose, and provide the autonomous interface |
| [SwerveModuleSubsystem.java](../src/main/java/frc/robot/subsystems/SwerveModuleSubsystem.java) | Convert one desired wheel speed and angle into motor requests |
| [HeadingController.java](../src/main/java/frc/robot/HeadingController.java) | Calculate a rotation correction or accept manual rotation |

A swerve module has two motors: propulsion spins the wheel; steering turns the wheel assembly. The four modules allow the robot to translate sideways and rotate at the same time.

## Coordinate frames

**Robot-relative** directions move with the robot. Positive x is forward, positive y is left, and positive angular velocity is counterclockwise under WPILib's convention. **Field-relative** directions stay aligned with the field coordinate axes.

If the robot faces 90 degrees counterclockwise from field x, moving along field +x requires robot-relative motion to its right, or −y. `ChassisSpeeds.fromFieldRelativeSpeeds(...)` performs that conversion.

This code uses raw gyro heading for driver field-relative movement and heading hold. Its estimated field pose may have a different heading offset after a pose reset or vision correction. Do not casually substitute `getPose().getRotation()` for `getRotation()`; they represent potentially different frames in this implementation.

## Joystick input, step by step

`TeleoperatedControlCommand` receives callbacks for x, y, and rotation. `initialize()` is empty. `execute()` does the following each cycle:

1. If teleop is not enabled, send zero chassis speeds and return.
2. Set each axis to zero if its absolute value is at most `0.15`. This is a **deadband** that ignores small stick drift.
3. Square the magnitude while preserving its sign: `Math.copySign(value * value, value)`. Small stick movements become gentler.
4. Multiply x and y by `maxVelocity`, currently 5.2 m/s.
5. Multiply rotation by `maxAngularVelocity`, currently `2π` rad/s.
6. Call `swerveSubsystem.generateSpeeds(...)`.

For illustration, an already sign-adjusted x input of `0.5` becomes `0.25`, then `1.3 m/s`. An input of `−0.5` becomes `−1.3 m/s`. This deadband does not rescale the remaining axis range; it simply replaces small values with zero.

`end()` sends zero chassis speeds when canceled or interrupted. `isFinished()` always returns false, so it runs until interrupted. Its drivetrain requirement prevents it from running alongside a normal drivetrain-requiring auto.

## Heading hold

`HeadingController` stores a target angle and a `locked` flag. Its PID uses the `thetaPID` constants and continuous input from −π to +π. Continuous input lets it choose the short correction across the angle wrap: 179° to −179° is approximately +2°, not −358°.

`calculate(heading, requestedOmega)` has two cases:

- If the requested rotation magnitude is at least `0.05 rad/s`, accept manual rotation and mark the heading unlocked.
- Otherwise, if unlocked, capture the current heading as the new target. Then calculate the PID correction toward the target.

The returned output is clamped to ±4 rad/s, including manual rotation. Therefore the nominal `maxAngularVelocity = 2π` scaling does not mean the driver can actually request `2π` through this controller.

`setTarget(angle)` stores an explicit target, resets PID history, and marks it locked. This allows buttons to request a heading even after manual turning. Entering teleop also captures the current gyro heading so an old target does not pull the robot back after autonomous.

This is a heading hold, not a trajectory planner. It does not impose a separately tuned angular acceleration profile.

## From chassis motion to four wheel states

`generateSpeeds(x, y, theta)`:

1. Passes rotation through `HeadingController`.
2. Converts field-relative translation to robot-relative `ChassisSpeeds` using the gyro.
3. Calls `ChassisSpeeds.discretize(..., 0.02)` to account for translation and rotation happening together over a 20 ms step.
4. Calls `setSpeeds()`.

`setSpeeds()` is also the PathPlanner output callback. That direct autonomous route bypasses driver input shaping and heading hold; PathPlanner supplies its own robot-relative chassis request.

`driveKinematics.toSwerveModuleStates(speeds)` calculates a speed and angle for each module from its position on the chassis. Conceptually, a wheel needs the robot's translation plus the local motion caused by rotation around the robot's center.

The code then **desaturates** wheel speeds. If any requested wheel exceeds the speed limit, all wheel speeds are reduced proportionally. This preserves their ratios. The limit is normally 5.2 m/s or 30% of that when `slowDown` is true.

`slowSpeed()` enables that lower ceiling; `regularSpeed()` removes it. Slow mode is a cap, not a multiplier applied to every small input. Its controller bindings are currently commented out.

The four states are sent in this fixed order: front left, front right, back left, back right. The same order is used by kinematics, odometry, and module arrays.

## Inside one swerve module

The constructor creates a propulsion TalonFX and steering TalonFX on `CANivore`, configures their output directions and limits, then creates a CANcoder. Steering feedback is configured as `FusedCANcoder`, using the encoder's ID and a rotor-to-sensor ratio of 12.8.

A **rotor-to-sensor ratio** describes how many motor rotations correspond to one sensor rotation. Configuring feedback ratios correctly is necessary for the controller to interpret steering position consistently.

The steering controller's internal slot-0 gains are P = 60, I = 0, D = 0.5, and static feedforward = 0.35. These are separate from the chassis heading gains. The propulsion controller uses a WPILib PID plus feedforward in Java instead of TalonFX velocity control.

### Startup alignment

`resetToAbsolute()` reads the CANcoder's absolute position, reverses its sign if requested, sets the steering position, and stores it as `lastAngleRotations`. Holding this last angle prevents the module from seeking an arbitrary zero angle on the first stop request.

The constructor receives and stores `encoderOffset`, but current calculations do not apply it. All four module constructors currently pass `encoderReversed = false`. Check actual CANcoder configuration before changing offset behavior: an offset may already be stored in the device.

### Distance and velocity conversion

The drive motor encoder measures motor rotations. To calculate wheel travel:

```text
wheel rotations = motor rotations / drive gear ratio
wheel circumference = 2 × π × wheel radius
distance in meters = wheel rotations × wheel circumference
```

The radius is 2 inches converted to meters, or 0.0508 m; the gear ratio is 5.9. With these constants, 5.9 motor rotations correspond to one wheel rotation, approximately 0.319 m of travel. The velocity conversion uses the same formula with rotations per second.

`getPropulsionPosition()` and `getPropulsionVelocity()` perform those conversions. They assume the reported propulsion position remains in motor rotations.

### Steering optimization and propulsion output

`setState(state, omegaRadPerSec)` receives a desired wheel speed and angle:

1. If wheel speed is below 0.05 m/s and chassis rotation magnitude is below 0.05 rad/s, hold the previous steering target and request zero propulsion voltage.
2. Otherwise, subtract current steering position from target angle.
3. Wrap that difference to a half-turn range, −0.5 to +0.5 rotations, to avoid unnecessary whole turns.
4. If the remaining steering move exceeds a quarter turn, choose the opposite wheel direction and negate drive speed. A wheel facing 180° away and driving backward can produce the same ground motion.
5. Add the chosen difference to current position to make an unwrapped steering target. Save it for later stopping.
6. Stop steering output when error is below 0.002 rotations (about 0.72°); otherwise request `PositionVoltage(unwrappedTarget)`.
7. Multiply drive speed by the cosine of steering error. A misaligned wheel contributes less propulsion while it turns into place.
8. Request propulsion voltage equal to PID feedback plus feedforward.

For illustration, if a wheel faces 0° and is asked to drive forward at a wheel angle of 170°, it can steer to −10° and reverse its wheel speed. This avoids a 170° steering movement.

`propulsionFF` uses `kS = 0.2625`, `kV = 2.5`, and `kA = 0`. In the current speed-only call, it estimates voltage from velocity. `propulsionPID` compares requested and measured wheel speed using gains 1.0, 0, and 0.01.

## Tracking the robot's pose

The drivetrain's `SwerveDrivePoseEstimator` combines gyro rotation and module travel with accepted camera observations.

Each drivetrain `periodic()`:

1. Calls `updateOdometryRobotRelative()` with gyro angle and all four measured module positions.
2. Calls `addVisionMeasurements()` to correct the estimate using accepted camera poses.
3. Updates the `Field2d` visualization and dashboard x, y, and heading.

**Odometry** estimates motion from changes in wheel travel and steering angle. It drifts because wheels slip and measurements are imperfect. Vision can correct that drift; it also needs filtering because camera estimates can be wrong.

The estimator starts at `new Pose2d()`, meaning field position (0, 0) and heading zero. Initial uncertainty settings are 0.05 m in x/y and 5° in heading for the process model, with default vision uncertainties of 0.7 m and 10°. Accepted vision measurements supply their own uncertainties later.

`autoPose(newPose)` resets the estimator using current sensors and a supplied field pose. It does not physically move the robot or reset the gyro. PathPlanner uses it to establish an auto's starting pose.

## Other drivetrain methods

| Method | Purpose and detail |
| --- | --- |
| `getRotation()` | Reads raw Pigeon yaw in degrees and returns a `Rotation2d`. |
| `resetHeading()` | Resets the gyro and requests zero heading hold. It does not explicitly reset the pose estimator; a gyro discontinuity can affect its next update. |
| `getPose()` | Returns the estimated field pose. |
| `getModuleStates()` | Returns measured wheel speed and absolute angle for all four modules. |
| `getRobotSpeeds()` | Converts measured module states back into robot-relative chassis speeds. |
| `setTargetRot(angle)` | Forwards an explicit gyro-relative heading target to the heading controller. |
| `humpRot()` | Selects `45 + 90 * floor(currentDegrees / 90)`, the diagonal in the current 90° sector. At 20° it selects 45°; at 110° it selects 135°. |
| `rotateToHub()` | Computes relative yaw from estimated pose to the configured hub, adds it to current gyro heading, and adds 180° before setting the target. |

The 180° addition in `rotateToHub()` points the robot's opposite-facing direction toward the target. It sets one target at button press; it is not continuous tracking. `hubCoords` is fixed at (4.625, 4.030) and this method does not switch hubs by alliance.

Module `getCANCoder()` returns the absolute encoder angle with optional sign reversal. `getTurnPosition()` returns TalonFX steering feedback. `getState()` combines measured drive velocity and CANcoder angle; `getPosition()` combines traveled distance and that angle. Module `periodic()` publishes encoder angles, plus an uninformative shared dashboard boolean literally named `getName()`.
