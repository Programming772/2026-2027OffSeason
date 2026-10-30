# 6. Constants, hardware, and project files

[Guide index](README.md) · [Previous: vision and autonomous](05-vision-and-autonomous.md) · [Next: development](07-development.md)

## Repository map

| Path | Role |
| --- | --- |
| `src/main/java/frc/robot/Main.java` | Java entry point |
| `src/main/java/frc/robot/Robot.java` | Robot lifecycle and scheduler |
| `src/main/java/frc/robot/RobotContainer.java` | Hardware ownership, controls, autonomous configuration |
| `src/main/java/frc/robot/Constants.java` | Shared configuration values |
| `src/main/java/frc/robot/HeadingController.java` | Driver rotation and heading hold |
| `src/main/java/frc/robot/VisionMeasurement.java` | Camera result data object |
| `src/main/java/frc/robot/commands/` | Teleop drive command |
| `src/main/java/frc/robot/subsystems/` | Drivetrain, module, vision, intake, and shooter implementations |
| `src/test/java/frc/robot/` | JUnit regression tests |
| `src/main/deploy/` | Files copied to the robot during deployment |
| `vendordeps/` | Vendor-library dependency descriptions |
| `.wpilib/wpilib_preferences.json` | Team number 772, Java, and project year 2026 |
| `build.gradle` | Build plugins, dependencies, testing, packaging, and deployment |
| `settings.gradle` | Gradle plugin resolution, including local WPILib Maven files |
| `gradlew`, `gradlew.bat`, `gradle/wrapper/` | Project's Gradle launcher and wrapper version configuration |
| `readme.md` | Project introduction and setup |
| `docs/` | This guide |
| `build/`, `.gradle/`, `bin/` when present | Generated output or caches; edit source files instead |
| `.git/` | Git history and repository metadata |

## `Constants.java`

[Constants.java](../src/main/java/frc/robot/Constants.java) groups values by subsystem. Nested classes are used as namespaces, allowing expressions such as `Constants.SwerveConstants.maxVelocity`.

A constant being declared does not mean it is used. Search for its name before assuming that editing it changes robot behavior. Examples include `pathConfig`, some acceleration limits, intake position alternatives, and steering offsets.

### Chassis and control values

| Constant group | Current values / meaning |
| --- | --- |
| `maxVelocity` | 5.2 m/s; driver translation scaling and normal wheel-speed ceiling |
| `maxAcceleration` | 1 m/s²; not currently applied by a teleop acceleration limiter |
| `maxAngularVelocity` | `2π` rad/s; joystick scaling before the heading controller's ±4 clamp |
| `maxAngularAcceleration` | `4π` rad/s²; not currently applied as a teleop acceleration limiter |
| `wheelRadiusMeters` | 2 inches converted to 0.0508 m |
| `kDriveGearRatio` | 5.9 motor rotations per wheel rotation |
| `kSteerGearRatio` | 12.8 rotor rotations per steering sensor rotation |
| `kTrackWidth`, `kWheelBase` | Both 22 inches, or 0.5588 m |
| `propulsionPID` | P/I/D = 1 / 0 / 0.01; wheel speed correction |
| `propulsionFF` | S/V/A = 0.2625 / 2.5 / 0; propulsion feedforward |
| `translationPID` | P/I/D = 10 / 0 / 0; autonomous position correction |
| `thetaPID` | P/I/D = 3 / 0 / 0; heading correction and autonomous rotation feedback |
| `turningPID` | All zero; currently not the steering controller's actual gains |
| `hubCoords` | Fixed field pose at x 4.625 m, y 4.030 m, heading 0° |
| `pathConfig` | A WPILib `TrajectoryConfig(3, 1)`; current PathPlanner setup does not consume it |

`driveKinematics` places modules at ±half the wheelbase in x and ±half the track width in y. Front left is (+x, +y), front right (+x, −y), back left (−x, +y), back right (−x, −y).

Actual steering slot gains are assigned directly in `SwerveModuleSubsystem`: changing the unused `turningPID` constants will not change those assignments.

### CAN device map

The bus is part of the address. “Default” below means the constructors do not explicitly select `CANivore`.

| Device | CAN ID | Bus |
| --- | --- | --- |
| Front-left drive / steer / CANcoder | 1 / 2 / 3 | `CANivore` |
| Front-right drive / steer / CANcoder | 4 / 5 / 6 | `CANivore` |
| Back-left drive / steer / CANcoder | 7 / 8 / 9 | `CANivore` |
| Back-right drive / steer / CANcoder | 10 / 11 / 12 | `CANivore` |
| Pigeon2 gyro | 13 | Default |
| Intake arm | 34 | Default |
| Intake roller | 35 | `CANivore` |
| Flywheel 1 | 31 | Default |
| Flywheel 2 | 0 | Default |
| Tower | 33 | `CANivore` |
| Feeder constant | 14 | No corresponding active controller is constructed |

These are source assignments, not verified wiring. In particular, validate flywheel 2's ID 0 against the robot rather than assuming it is either correct or invalid.

The climber solenoid uses PCM channels 2 and 4. Those channels are pneumatic-controller outputs, not motor CAN IDs.

### Steering offsets and inversion

Stored offset constants, in rotations, are front left 0, front right −0.3515625, back left 1.142578125, and back right 0.52734375. They are passed into module constructors but currently not applied in calculations.

Left propulsion motors use clockwise-positive configuration, right propulsion motors counterclockwise-positive. All steering motors use counterclockwise-positive. All `encoderReversed` arguments are false.

An incorrect sign can make feedback reinforce error instead of reducing it. An incorrect offset can make the controller believe a wheel points forward when it points elsewhere. Establish actual hardware configuration before changing either.

### Mechanism and vision values

Intake position constants include `intakePos = −1.28`, `restPos = −0.75`, `horizontalPos = 1.43`, and `ground = 1.7`. The active driver bindings use the first two; test-controller release targets zero. `flywheelTest = 4000` is declared but does not activate a shooter command.

Vision uses minimum 2 detected tags, maximum best-target ambiguity 0.2, maximum average distance 5 m, and maximum pose jump 1.5 m. Base uncertainty factors are 0.05 for x/y and 0.1 for heading. See [vision](05-vision-and-autonomous.md) for how these are used rather than treating them as independent switches.

## PathPlanner robot settings

[settings.json](../src/main/deploy/pathplanner/settings.json) currently contains robot mass 56 kg, moment of inertia 5.377 kg·m², wheel radius 0.05 m, drive gearing 5.9, maximum drive speed 5.2 m/s, Kraken X60 motor type, drive current limit 60 A, and wheel coefficient of friction 1.2.

Moment of inertia describes resistance to angular acceleration. Wheel friction affects the modeled ability to accelerate without slipping. These are physical-model inputs, not feedback gains.

Module x/y offsets in that file are ±0.305 m. The Java kinematics use ±0.2794 m from a 22-inch wheelbase/track width. The wheel radii also differ slightly: 0.05 m in the GUI file versus 0.0508 m in Java. These differences should be checked against measured hardware. Documentation cannot determine which geometry is correct.

GUI defaults and individual path constraints are separate from Java constants. Editing `maxAcceleration` in Java does not automatically rewrite saved path files.

## Build and dependencies

[build.gradle](../build.gradle) applies Java and GradleRIO 2026.2.1 and targets Java 17. `ROBOT_MAIN_CLASS` is `frc.robot.Main`.

The dependency blocks request WPILib and vendor Java libraries plus native libraries for the roboRIO and desktop testing/simulation. **Native libraries** contain platform-specific compiled code, so compiling Java alone does not prove desktop simulation can load all required libraries.

Vendor manifests currently specify Phoenix 6 26.1.0, Phoenix 5 5.36.0, PhotonVision `v2026.1.1-rc-4`, and PathPlanner 2026.1.2. Their JSON files tell Gradle where to obtain compatible artifacts. The `WPILibNewCommands.json` version label is a manifest version, not a reason to replace the project's WPILib version.

JUnit Jupiter 5.10.1 is the testing library. The `test` task uses JUnit Platform and WPILib's test-task configuration.

The `jar` task bundles Java dependencies into the robot application and includes backup copies of source, vendor manifests, and the build file. Deployment uploads that application and copies `src/main/deploy` to `/home/lvuser/deploy`.

Static deployment currently sets `deleteOldFiles = false`. Removing a path from your local repository may therefore leave an old deployed file on the robot. Keep that in mind when investigating stale autonomous choices.

`includeDesktopSupport = false` is declared in the build script but not referenced later. Actual desktop support depends on the configured tasks, dependencies, and implemented simulation, not merely that unused variable.
