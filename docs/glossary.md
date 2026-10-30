# Robotics and project glossary

[Guide index](README.md)

| Term | Meaning in this project |
| --- | --- |
| Absolute encoder | Sensor reporting an angle within a rotation without accumulating travel from a startup zero; the steering CANcoders provide this. |
| Alliance flipping | PathPlanner's transformation of autonomous field geometry according to the alliance callback. |
| AprilTag | Visually identifiable field marker with a known field pose. |
| Auto / autonomous | Enabled robot mode in which a selected command routine controls motion. |
| CAN | Device communication bus; IDs address devices on a particular bus. |
| CANcoder | CTRE absolute encoder used for steering feedback. |
| CANivore | Named CAN interface used by the drivetrain and selected mechanism controllers. |
| Chassis speeds | Robot translation velocities and angular velocity, represented by `ChassisSpeeds`. |
| Closed loop | Control that uses a measured quantity to correct error from a target. |
| Command | WPILib action with scheduling, execution, completion, and interruption behavior. |
| Coprocessor | Computer performing additional processing; PhotonVision can solve tag poses there. |
| Deadband | Input range mapped to zero to suppress small joystick drift. |
| Default command | Command scheduled when a subsystem has no other command requiring it. It is not inherently teleop-only. |
| Desaturation | Proportional scaling of wheel speeds so none exceeds the configured ceiling. |
| Discretization | Adjusting a continuous motion request for execution over a finite loop interval. |
| Driver Station | Operator software that provides mode, enable, alliance, and joystick information. |
| Duty cycle / output fraction | Normalized motor output request, such as 0.4; not a measured speed. |
| Feedforward | Estimate of required output based on requested motion rather than measured error. |
| Feedback | Correction based on measured error. |
| Field-relative | Expressed in fixed field axes rather than axes that turn with the robot. |
| FusedCANcoder | CTRE feedback mode combining rotor sensing with a remote CANcoder reference. |
| HAL | WPILib hardware abstraction layer; also used by simulated Driver Station tests. |
| Heading / yaw | Planar orientation about the vertical axis. |
| Holonomic | Able to control planar translation and rotation independently; swerve supports this. |
| Homing | Establishing a physical reference using a repeatable known position or sensor. Setting an encoder to zero alone is not homing. |
| Kinematics | Conversion between chassis motion and individual wheel motion using geometry. |
| Motion Magic | CTRE onboard profiled position-control mode; configured but not used by the active intake position bindings. |
| Neutral mode | Controller behavior at neutral output, such as brake or coast. |
| Odometry | Estimating robot movement from wheel and gyro measurements. |
| Open loop | Direct output request without regulation of a measured target quantity. |
| PCM | Pneumatics Control Module used to drive the climber valve. |
| PID | Feedback control using proportional, integral, and derivative error terms. |
| Pigeon2 | CTRE gyro used to measure yaw. |
| Pose | Position and orientation together. |
| Pose estimator | Component combining motion measurements and vision into an estimated field pose. |
| Requirement | Subsystem ownership declared by a command for scheduler conflict handling. |
| roboRIO | Onboard computer running the robot Java application. |
| Robot-relative | Expressed in axes fixed to the robot body. |
| Rotor | Rotating part of the motor; rotor rotations differ from wheel rotations through gearing. |
| RPM / RPS | Revolutions per minute / rotations per second; RPM = RPS × 60. |
| Setpoint | Requested target position, speed, or heading. |
| Slot 0 | One set of CTRE onboard closed-loop gains selected by control requests. |
| SmartDashboard | Dashboard interface used for published telemetry and the autonomous chooser. |
| Standard deviation | Uncertainty measure used to set the relative influence of sensor observations. |
| Subsystem | WPILib component grouping hardware operations and periodic behavior. |
| Swerve module | Independently propelled and steered wheel assembly. |
| TalonFX | CTRE motor controller interface used throughout the robot. |
| Telemetry | Reported measurements and state; publishing it does not itself control a mechanism. |
| Teleop | Driver-controlled enabled mode. |
| Trajectory | Planned motion over time, including positions, velocities, and constraints. |
| Transform | Translation and rotation relating two coordinate frames. |
| Trigger | Scheduler-polled condition used to schedule or cancel commands on transitions. |
| WPILib | FRC library providing runtime, hardware abstraction, commands, math, and tools. |
