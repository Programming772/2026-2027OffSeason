# 5. Vision and autonomous

[Guide index](README.md) · [Previous: mechanisms](04-mechanisms.md) · [Next: configuration](06-configuration.md)

## What vision contributes

Wheel-and-gyro odometry tracks relative motion but can drift. AprilTags are square visual markers with known field locations. If the camera sees suitable tags and knows its mounting position, PhotonVision can estimate where the robot is on the field.

The main pieces are:

- [VisionSubsystem.java](../src/main/java/frc/robot/subsystems/VisionSubsystem.java): camera access, pose estimation, and initial filtering.
- [VisionMeasurement.java](../src/main/java/frc/robot/VisionMeasurement.java): a small object carrying one estimate and its quality information.
- `SwerveDriveSubsystem.addVisionMeasurements()`: final filtering and fusion into the drivetrain's pose estimator.

The camera observation is a measurement, not a motor command. It affects driving indirectly by changing the estimated pose used by autonomous and hub-heading calculations.

## Camera setup

The vision constructor loads the default AprilTag field layout from the installed WPILib version. It creates one `PhotonCamera` and one `PhotonPoseEstimator` for each camera name in `VisionConstants`.

Currently there is one camera named `Turret`. The corresponding transform places it at x = 0, y = 0, and z = 18.75 inches above the robot origin, with zero roll, pitch, and yaw. `Units.inchesToMeters()` converts the height before constructing `Translation3d`.

The name and transform arrays must stay aligned: element 0 of each describes the same camera. A camera's configured PhotonVision name must match the string in the code.

The pose estimator prefers `MULTI_TAG_PNP_ON_COPROCESSOR`: use a pose solution calculated from several tags by the vision computer. Its fallback is `LOWEST_AMBIGUITY`. Ambiguity is a measure of how uncertain the tag pose interpretation is; lower is generally preferable.

A camera named `Turret` is not automatically modeled as moving. The current transform is fixed. If the physical camera rotates with a turret, this transform must reflect that changing orientation for accurate robot poses.

## Reading and filtering frames

`getVisionMeasurements()` creates a new list, then processes each camera:

1. Call `getAllUnreadResults()` once to drain its queued frames. This avoids repeatedly reading only the latest frame and skipping intermediate observations.
2. Reject a frame with no targets or fewer than `MIN_TAGS`, currently 2 detected targets.
3. Reject it if the best target's ambiguity exceeds 0.2.
4. Calculate average tag distance by adding each camera-to-target translation's length and dividing by target count.
5. Reject it if that average exceeds 5 m.
6. Ask the Photon pose estimator to update from this frame.
7. If a pose is returned, add a `VisionMeasurement` containing the estimate, average distance, and detected target count.
8. After all cameras, sort the measurements by timestamp so observations from different queues are returned chronologically.

`estimator.update()` can return an empty `Optional` even when targets are detected; only usable pose estimates become measurements.

The initial tag count and average distance are calculated from detected targets, not necessarily only the tags ultimately used by the pose solver. The ambiguity check examines the best target, not every tag. These are current filtering choices, not a complete guarantee of estimate quality.

Although the estimator has a fallback strategy, the earlier minimum-target check still rejects all single-target frames.

## What `VisionMeasurement` stores

| Field | Meaning |
| --- | --- |
| `pose` | PhotonVision's `EstimatedRobotPose`, including a 3D robot pose and timestamp |
| `avgTagDistance` | Average detected tag distance in meters |
| `tagCount` | Number of detected targets in that frame |

This is a data carrier with `public final` fields; it performs no filtering or fusion.

## Fusing a camera pose with odometry

The drivetrain first updates wheel/gyro odometry, then processes the returned vision list:

1. Convert the estimated 3D pose into a `Pose2d` for floor-plane driving.
2. Compare its translation to the current estimated translation.
3. Reject it if that distance exceeds 1.5 m (`MAX_POSE_JUMP_METERS`).
4. Compute uncertainty values and call `poseEstimator.addVisionMeasurement()` with the pose, camera timestamp, and uncertainties.

The timestamp describes when the observation was captured, not simply when Java received it. Passing it allows the pose estimator to account for measurement delay within its history. Valid time synchronization and timely data still matter.

A **standard deviation** expresses uncertainty. Smaller values mean the measurement should receive greater trust. Current calculations are:

```text
xy standard deviation = 0.05 × average tag distance
heading standard deviation = 0.1 × average tag distance
if more than one tag: divide each by tag count
```

For two tags at an average 3 m distance, x/y uncertainty becomes 0.075 m and heading uncertainty becomes 0.15 radians, about 8.6°. This is a heuristic: more tags increase trust; greater distance reduces it.

The 1.5 m jump limit has a tradeoff. It rejects large outliers, but it can also prevent recovery when the initial pose is badly wrong. The estimator starts at field (0, 0), and `autoPose()` can establish a known starting pose. Also, the jump test compares a potentially delayed camera pose against the current estimate rather than a historical estimate at the same timestamp. Fast motion and delay can affect acceptance.

## Autonomous configuration

`RobotContainer.configureAutoBuilder()` supplies PathPlanner with callbacks:

| Callback or parameter | Purpose |
| --- | --- |
| `swerveSubsystem::getPose` | Read estimated field position and heading |
| `swerveSubsystem::autoPose` | Reset the estimated pose to an auto's starting pose |
| `swerveSubsystem::getRobotSpeeds` | Read measured robot-relative chassis velocity |
| `swerveSubsystem::setSpeeds` | Apply requested robot-relative chassis velocity |
| `PPHolonomicDriveController` | Correct translation and heading errors while following the planned motion |
| Loaded `RobotConfig` | Describe mass, inertia, drive hardware, and module geometry |
| Alliance callback | Return true for red alliance so PathPlanner applies its field flipping behavior |
| `swerveSubsystem` requirement | Give the auto exclusive command ownership of the drivetrain |

If alliance is unknown, the callback defaults to blue. PathPlanner's coordinate system remains field-based; alliance flipping is handled through its configuration rather than manually negating joystick inputs.

The holonomic controller uses `translationPIDkp/ki/kd` (10, 0, 0) and `thetaPIDkp/ki/kd` (3, 0, 0). Translation feedback responds to position error; rotation feedback responds to heading error. These are distinct from wheel speed and steering-motor gains.

A path's planned velocity supplies the intended motion; feedback corrects deviations. The current gains need real-robot tuning even though their wiring now uses the intended constant groups.

## `.auto` versus `.path`

Files live under [src/main/deploy/pathplanner](../src/main/deploy/pathplanner). Their JSON schemas separate path geometry, command composition, and robot configuration.

- A `.path` describes geometry and motion constraints.
- An `.auto` describes a sequence or composition of commands, such as following a path and then running a named command.
- `settings.json` describes the robot and GUI defaults.
- `navgrid.json` describes grid-based navigation information. This Java code does not currently call a pathfinding API just because that file exists.

Use the matching PathPlanner application to edit these files. A serialized file version such as `2025.0` is a schema identifier; do not replace it solely because the project uses 2026 libraries.

### Example: `Test Line`

[Test Line.auto](<../src/main/deploy/pathplanner/autos/Test Line.auto>) contains a sequential group with one path command referencing `Test Line`. `resetOdom: true` requests an initial odometry reset for that auto.

[Test Line.path](<../src/main/deploy/pathplanner/paths/Test Line.path>) goes from anchor (1, 6) to (3, 6), in meters. Control points shape the curve between the anchors; here they form a straight horizontal line. The path has:

- Maximum speed 3 m/s and maximum acceleration 3 m/s².
- Angular speed/acceleration fields of 540°/s and 720°/s² in the serialized path format.
- Starting and ending velocity zero and heading zero.
- No event markers, intermediate rotation targets, or constraint zones.

The robot's heading can be independent of the direction of travel because swerve can translate sideways. An anchor position describes where to go; a rotation target describes how to face while going there.

### Current autonomous choices

| Auto file | Referenced path / extra command |
| --- | --- |
| `New Auto.auto` | `Example Path` |
| `New New Auto.auto` | `New New Path` |
| `Test Line.auto` | `Test Line` |
| `Test L.auto` | `Test L Path` |
| `Test Turn.auto` | `Test Turn` |
| `We suck.auto` | `We suck` |
| `Shoot & Climb R.auto` | `Shoot & Climb R`, then named command `Shoot` |

The Java code does not register a named `Shoot` command. That auto therefore does not have an implemented shooter action behind that name. It also does not contain an implemented climber command. Validate both file references and named-command registration before relying on an autonomous routine.

## From dashboard selection to movement

`AutoBuilder.buildAutoChooser()` creates a chooser published as `Auto Chooser`. `getAutonomousCommand()` returns its selected command. In `autonomousInit()`, `Robot` schedules that selection if it is non-null.

During the auto, PathPlanner repeatedly reads pose and speeds, calculates a requested motion, and calls `setSpeeds()`. The drivetrain converts that request into wheel states. On entering teleop, `Robot` cancels the auto and resets heading hold to the current gyro heading. The default joystick command can then regain the drivetrain.
