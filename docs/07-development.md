# 7. Building, testing, and making changes

[Guide index](README.md) · [Previous: configuration](06-configuration.md) · [Glossary](glossary.md)

## Build environment

Use the 2026 WPILib development environment for this project. The Gradle build uses GradleRIO 2026.2.1 and targets Java 17. The wrapper supplies the configured Gradle version; WPILib and vendor dependencies must also be available.

From the repository root:

```sh
./gradlew compileJava
./gradlew test
./gradlew build
```

On Windows, use `.\gradlew.bat` instead. If the Unix wrapper lacks execute permission, `bash ./gradlew test` invokes it through Bash.

| Task | What it establishes |
| --- | --- |
| `compileJava` | Main Java sources compile against the resolved dependencies |
| `test` | Test sources compile and the JUnit suite runs with configured native dependencies |
| `build` | Runs the project's build lifecycle, including assembly and checks |

A successful build does not verify CAN wiring, encoder references, motor direction, mechanical travel, camera calibration, or tuning.

If `java` is unavailable, select the installed WPILib JDK rather than changing the project's target version. On the development machine used for this review, this command works:

```sh
JAVA_HOME=/home/nav/wpilib/2026/jdk ./gradlew test --offline
```

That absolute path is machine-specific. `--offline` requires all needed Gradle artifacts to be cached; omit it when dependencies need downloading. Gradle also needs write access to its cache for lock files, even when offline.

The current vision implementation compiles with deprecation warnings for several pose-estimator and field-layout APIs. Those warnings are migration work, not evidence that the current compilation failed. Coordinate vendor API upgrades with the project's dependency versions.

## Existing tests and their scope

Tests live in [src/test/java/frc/robot](../src/test/java/frc/robot). At the time of this documentation, the suite contains nine tests.

### `HeadingControllerTest`: six tests

[HeadingControllerTest.java](../src/test/java/frc/robot/HeadingControllerTest.java) exercises the heading controller without constructing the drivetrain:

- Corrections have the appropriate sign on either side of a target.
- Angle wrap uses the shortest correction across ±180°.
- Releasing manual rotation captures the current heading.
- An explicit target survives the transition out of manual rotation.
- Resetting a target does not return to an old heading.
- Manual and corrective rotation output are clamped to ±4 rad/s.

These verify the controller's numerical behavior. They do not exercise a physical gyro or steering module, nor do they directly test `Robot.teleopInit()`.

### `IntakeRollerBindingsTest`: three tests

[IntakeRollerBindingsTest.java](../src/test/java/frc/robot/IntakeRollerBindingsTest.java) calls the same static binding helper used by `RobotContainer`, supplies simulated button conditions, and records the requested output through a `DoubleConsumer`.

It initializes HAL (WPILib's hardware abstraction layer), sets simulated Driver Station state, and runs the real command scheduler. This tests command and trigger semantics without constructing the intake motor controllers.

The tests verify:

- Releasing one controller's button does not stop the roller while the other remains held.
- Disabling stops output, and releasing while disabled does not leave a stale request on re-enable.
- A held button cannot run the roller in autonomous but becomes active in enabled teleop.

Each test clears scheduler bindings and resets simulated Driver Station state to avoid leakage through the shared scheduler. The tests check requested output, not roller RPM or electrical behavior.

### Test reports and simulation limits

Gradle writes its HTML report to `build/reports/tests/test/index.html` and XML results under `build/test-results/test/`.

The build includes desktop native dependencies and simulation configuration. There is no complete drivetrain or mechanism physics simulation in the source. Simulating Driver Station transitions is useful for command logic, but a desktop run will not automatically reproduce wheel motion, arm gravity, or camera observations.

Useful future regression tests include module optimization boundaries, vision filtering and timestamp order, autonomous controller setup, and pose behavior around gyro resets. Isolate computations or inject hardware boundaries where doing so improves the design; avoid constructing the entire hardware graph just to test a calculation.

## How to trace a behavior

Start from the entry point for the behavior and follow data, ownership, and units:

1. Find the binding, default command, or autonomous callback in `RobotContainer`.
2. Check the scheduling condition and subsystem requirements.
3. Follow the called subsystem method to the motor request or estimator update.
4. Identify every unit conversion and coordinate-frame conversion.
5. Find the feedback configuration that determines the meaning of sensor values.
6. Check cancellation, button release, and mode transitions.
7. Find the dashboard measurement that can confirm the expected state.

For an intake position issue, trace `intakePositionCommand()` → `moveIntake()` → `getIntakePosition()` / `manualIntakePosition()`. For an auto tracking issue, trace the pose and speed callbacks separately from `setSpeeds()` and module outputs. A wrong pose can produce incorrect control even when the output conversion is correct.

## Making common changes

### Add or change a controller binding

Identify the Driver Station slot and raw button number. Choose an edge-triggered action for a one-time state change, or a command with repeated execution and cleanup for ongoing motion. Declare requirements when the action must exclude other commands.

Define mode behavior explicitly. A label such as “test controls” in a comment does not restrict a binding to test mode. Remember that `whileTrue` does not restart a command that finishes or is interrupted while its condition stays true.

If adding roller control, account for the existing combined binding. The arm and roller currently share a subsystem, while the roller command deliberately has no arm requirement. A second independent motor writer would bypass that coordination.

### Change a position or speed

Find the actual active call path first. Editing Motion Magic parameters will not change `moveIntake()`'s fixed-output logic. Editing `turningPID` constants will not change the slot gains assigned directly in the module constructor. Editing Java acceleration constants will not automatically change PathPlanner's saved constraints.

Document the units, source of calibration, and intended reference position. For arm positions, establish whether the startup zero corresponds to a repeatable physical pose. For wheel geometry, reconcile Java kinematics and PathPlanner settings with the same measured dimensions.

### Add an autonomous mechanism action

The current code does not register named mechanism commands. An auto reference such as `Shoot` needs a registered implementation before it can perform the intended action.

A completed shooting command would need an explicit lifecycle: request flywheel speed, establish a readiness condition, control feeding, and stop or transition outputs on completion or interruption. Its gains, sensor interpretation, and ownership must be implemented before connecting the name in PathPlanner. A method that sends an RPM request is only one piece of that behavior.

### Add a camera

Add matching entries in `cameraNames` and `cameraPositions`, with the transform measured relative to the same robot origin and axis convention. Confirm PhotonVision's configured name, camera calibration, and tag layout. Reassess filtering and uncertainty with the actual camera placement.

For a moving mount, a fixed transform is insufficient. The robot-to-camera transform must follow the mount's measured pose using an appropriate estimator integration.

## Deployment

Deployment replaces the application on the roboRIO. The team number is read from `.wpilib/wpilib_preferences.json`, currently 772. Use WPILib's deployment workflow with the robot connected and in a controlled test state.

The application JAR and static files have separate deployment configuration. Static files go to `/home/lvuser/deploy`, and old files are not automatically deleted. If an obsolete auto still appears, compare deployed files with the local `src/main/deploy/pathplanner` tree.

This documentation task does not deploy anything. Code and test validation should be followed by mechanism and drivetrain checks appropriate to the changes before using a revised program on the field.

## Troubleshooting by symptom

| Symptom | Code or configuration to inspect |
| --- | --- |
| No Java executable | Installed WPILib JDK and `JAVA_HOME` / PATH |
| Gradle cannot resolve a library | Dependency cache, network access, GradleRIO version, and vendor manifest repositories |
| Native-library loading failure in tests | Desktop native artifacts and WPILib test-task configuration |
| Controller does nothing | Driver Station slot, raw button/axis value, active binding, mode gate, command ownership |
| Arm stops when another arm button is pressed | Shared intake requirement and `whileTrue` interruption semantics |
| Arm moves in the wrong direction | Motor inversion, feedback direction, startup zero, target units |
| RPM setter produces no expected motion | Zero or missing velocity gains, selected slot, bus and device configuration |
| Wheels steer to unexpected angles | CANcoder calibration, unused source offsets, feedback ratio, inversion, physical module order |
| Driver motion uses an unexpected field direction | Raw gyro heading and reset behavior; driver conversion does not use estimated field heading |
| Auto drifts or oscillates | Pose accuracy, geometry mismatch, translation/theta gains, wheel limits, path constraints |
| Vision never changes pose | Camera name, minimum tag count, ambiguity/distance filters, 1.5 m jump rejection, transform, tag layout |
| Hub aim chooses the wrong direction | Fixed hub coordinates, 180° addition, and lack of alliance-aware hub selection |
| Old autonomous choices remain | Static deployment's `deleteOldFiles = false` |
| `Shoot & Climb R` does not shoot | Missing registration and implementation of named command `Shoot` |

## Known limitations to preserve in reviews

These observations describe the current implementation; they are not all resolved by passing tests:

- Shooter velocity gains are zero, and shooter/turret/hood controls are not actively bound.
- Intake Motion Magic constraints and feedback setup are incomplete; active positioning uses fixed output instead.
- The arm is zeroed at startup without homing, and the active position controller has no explicit soft travel limits.
- Steering offset constants are stored but unused. Check device-side calibration before applying them in software.
- Java and PathPlanner module geometry differ, as do their configured wheel radii.
- Resetting the gyro does not explicitly preserve or rebase the pose estimator's field pose.
- Several arm and heading button bindings are not restricted to teleop.
- Hub aiming uses a fixed hub pose and a one-time heading target, with no alliance selection.
- Vision uses a fixed camera transform and can reject valid observations when the starting estimate is too far away.
- `Shoot & Climb R` references an unregistered named command.
- Hardware configuration calls do not systematically check status codes or retry failures.
- Full physical simulation and end-to-end hardware validation are not implemented by the unit tests.

When changing these areas, record the intended behavior and evidence for hardware assumptions. Keep this guide aligned with active code rather than inferred behavior from class, trigger, or auto names.
