# 4. Intake, shooter, and climber

[Guide index](README.md) · [Previous: swerve](03-swerve-drive.md) · [Next: vision and autonomous](05-vision-and-autonomous.md)

## Intake hardware and configuration

[IntakeSubsystem.java](../src/main/java/frc/robot/subsystems/IntakeSubsystem.java) owns two TalonFX controllers:

- `intakeArm`: ID 34 on the default CAN bus, moves the arm.
- `intakeRoller`: ID 35 on `CANivore`, spins the roller.

The constructor sets the arm's reported position to zero. This is **not homing**: no limit switch or physical reference is checked. Zero means the startup position unless hardware or an operator establishes a known starting pose.

The arm is configured for brake mode and clockwise-positive output. The roller uses coast mode and counterclockwise-positive output. Brake mode electrically resists motor movement when neutral; it is not a mechanical lock. Coast mode allows freer motion when neutral.

Both controllers receive fresh default configurations followed by the custom configurations. This means source configuration can overwrite settings stored on the controllers. Voltage peaks are ±12 V. Current-limit values are also assigned, but the code does not explicitly set every related enable flag; a numeric limit alone is not proof of the effective device limit.

## The active arm position algorithm

`moveIntake(setPoint)` is the method used by active position bindings. It:

1. Saves the target in `intakeSetpoint`.
2. Reads `getIntakePosition()`.
3. Stops if the current position is within 0.05 of the target.
4. Otherwise applies output ±0.2 in the direction that reduces the error.

The direction expression is:

```java
-0.2 * ((currentPos - intakeSetpoint) / Math.abs(currentPos - intakeSetpoint))
```

For a nonzero difference, dividing a value by its absolute value produces +1 or −1. The minus sign makes the motor move toward the target, assuming feedback direction matches motor direction. The near-target check handles zero error before this division.

Example: current position 0 and target −1.28 produce a positive error of 1.28, so output is −0.2. Near −1.28, the method requests zero. If it overshoots far enough, the sign reverses.

This is a simple fixed-output position controller. It does not slow proportionally as it approaches the target, enforce software travel limits, or guarantee gravity compensation. The target values are reported controller-position units; because the arm does not configure a mechanism gear ratio, do not assume they are physical arm rotations or degrees.

The method must be called repeatedly. `intakePositionCommand()` in `RobotContainer` does that and sends zero when the command ends. The `intakeSetpoint` field by itself does not cause `periodic()` to keep moving the arm.

## Intake method reference

| Method | Current behavior |
| --- | --- |
| `setZero()` | Sets the reported arm position to zero without moving or homing it. |
| `getIntakePosition()` | Reads TalonFX position as a `double`. |
| `manualIntakePosition(speed)` | Directly requests an arm output fraction. |
| `moveIntake(setPoint)` | Performs one position check and requests −0.2, 0, or +0.2. |
| `setIntakeArmPosition(targetPosition)` | Requests Motion Magic position control using slot 0; active bindings do not call this method. |
| `manualIntakeRoller(speed)` | Directly requests a roller output fraction. |
| `setIntakeRollerRPM(targetRPM)` | Converts RPM to rotations per second and requests `VelocityVoltage`. |
| `getIntakeRollerRPM()` | Converts measured rotations per second to RPM. |
| `periodic()` | Applies a conditional arm stop and publishes arm position and roller RPM. |

### Motion Magic is an unfinished alternative

Motion Magic is CTRE's onboard profiled position control. It needs suitable position gains, motion constraints, feedback units, and mechanism calibration.

The arm configuration sets cruise velocity, acceleration, and jerk to zero. It sets gravity feedforward `kG = 1.85` with `Arm_Cosine`, but no tuned position PID gains. The active manual-position routine does not use that closed-loop gravity configuration.

`periodic()` checks whether the arm is within 0.1 of `intakePos` and whether the stored Motion Magic request's target exactly equals `intakePos`. If both are true, it stops the arm. Active `moveIntake()` calls do not update that Motion Magic target. This check is therefore not the stopping mechanism for the currently bound position commands. If Motion Magic is enabled later, reconsider how this stored request interacts with manual control.

The roller velocity-control method also lacks tuned velocity gains in the roller configuration. Active buttons use 0.4 open-loop output instead of an RPM target.

## Shooter hardware and methods

[ShooterSubsystem.java](../src/main/java/frc/robot/subsystems/ShooterSubsystem.java) owns:

| Controller | ID / bus | Output direction |
| --- | --- | --- |
| `flywheel1` | 31 / default | Clockwise positive |
| `flywheel2` | 0 / default | Counterclockwise positive |
| `tower` | 33 / `CANivore` | Counterclockwise positive |

All use coast mode, ±12 V peaks, and configured stator/supply current-limit values of 100 A and 60 A. The code applies a default configuration before each custom configuration.

The two flywheel controllers are independently controlled. There is no follower relationship that automatically makes one mirror the other.

| Method family | Meaning |
| --- | --- |
| `setflywheel1RPM`, `setflywheel2RPM`, `setTowerRPM` | Divide the target RPM by 60 and send a slot-0 `VelocityVoltage` request. The lowercase `f` in the flywheel setter names is part of the actual API. |
| `getFlywheel1RPM`, `getFlywheel2RPM`, `getTowerRPM` | Read controller velocity and multiply by 60. |
| `manualFlywheel1`, `manualFlywheel2`, `manualTower` | Request an output fraction directly. |
| `periodic()` | Publish all three measured RPM values to SmartDashboard. |

All configured shooter velocity PID and feedforward gains are zero. An RPM request therefore lacks tuned control terms to produce the desired speed. A setter existing in Java does not mean the mechanism is ready to use.

There are currently no active shooter button bindings, no automatic spin-up or “ready to shoot” logic, and no registered `Shoot` named command. Turret and hood trigger names appear in `RobotContainer`, but this subsystem does not create turret or hood motors.

## Climber

`RobotContainer` creates a `DoubleSolenoid` using a CTRE PCM and channels 2 and 4. A double solenoid has forward, reverse, and off valve states. The constructor requests `Value.kForward`; whether this physically extends or retracts the climber depends on plumbing.

The driver toggle binding is commented out. The Java program currently has no active climber toggle command, position feedback, or climb sequence. An auto filename containing “Climb” does not create one.

## Dashboard data

These keys help relate requests to measurements:

| Key | Measurement |
| --- | --- |
| `Intake Pos` | Reported arm position |
| `Intake RPM` | Measured roller RPM |
| `Flywheel1 RPM` | First flywheel speed |
| `Flywheel2 RPM` | Second flywheel speed |
| `Tower RPM` | Tower speed |

Dashboard output is telemetry: a report of program state. Publishing an RPM does not regulate it, and a plausible number does not prove a mechanism is mechanically calibrated.
