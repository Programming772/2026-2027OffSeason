// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;


public final class Constants {
  // object for trajectory generation configs
  public static final TrajectoryConfig pathConfig = new TrajectoryConfig(3, 1);

  public static final Pose2d hubCoords = new Pose2d(4.625, 4.030, Rotation2d.fromDegrees(0));


  public final class SwerveConstants {
    public static final double maxVelocity = 5.2;
    public static final double maxAcceleration = 1;
    public static final double maxAngularVelocity = 2 * Math.PI;
    public static final double maxAngularAcceleration = 4 * Math.PI;

    // physical constants of the swerve drive modules
    public static final double wheelRadiusMeters = Units.inchesToMeters(2);
    public static final double kDriveGearRatio = 5.9;
    public static final double kSteerGearRatio = 12.8;

    public static final int pigeonID = 13;

    // constants for front left module
    public static final class FrontLeftConstants {
      public static final int frontLeftPropulsionID = 1;
      public static final int frontLeftTurningID = 2;
      public static final int frontLeftCANCoderID = 3;
      public static final Angle frontLeftCANCoderOffset = Rotations.of(0);
    }

    // constants for front right module
    public static final class FrontRightConstants {
      public static final int frontRightPropulsionID = 4;
      public static final int frontRightTurningID = 5;
      public static final int frontRightCANCoderID = 6;
      public static final Angle frontRightCANCoderOffset = Rotations.of(-0.3515625);
    }

    // constants for back left module
    public static final class BackLeftConstants {
      public static final int backLeftPropulsionID = 7;
      public static final int backLeftTurningID = 8;
      public static final int backLeftCANCoderID = 9;
      public static final Angle backLeftCANCoderOffset = Rotations.of(1.142578125);
    }

    // constants for back right module
    public static final class BackRightConstants {
      public static final int backRightPropulsionID = 10;
      public static final int backRightTurningID = 11;
      public static final int backRightCANCoderID = 12;
      public static final Angle backRightCANCoderOffset = Rotations.of(0.52734375);
    }

    // propulsion PID coefficients
    public static final double propulsionPIDkp = 1.0;
    public static final double propulsionPIDki = 0.0;
    public static final double propulsionPIDkd = 0.01;

    // propulsion FF coefficients
    public static final double propulsionFFka = 0.0;
    public static final double propulsionFFks = 0.2625;
    public static final double propulsionFFkv = 2.5;

    // turning PID coefficients
    public static final double turningPIDkp = 0;
    public static final double turningPIDki = 0.0;
    public static final double turningPIDkd = 0.0;
    public static final double translationPIDkp = 10.0;
    public static final double translationPIDki = 0.0;
    public static final double translationPIDkd = 0.0;

    // theta PID coefficients
    public static final double thetaPIDkp = 3.0;
    public static final double thetaPIDki = 0.0;
    public static final double thetaPIDkd = 0.0;

    // distance between modules
    public static final double kTrackWidth = Units.inchesToMeters(22);
    public static final double kWheelBase  = Units.inchesToMeters(22);

    // creates a SwerveDriveKinematics object using the locations of the modules relative to centre of the robot
    public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics (
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),   // front left module
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),  // front right module
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),  // back left module
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)  // back right module
    );
  }


  public static final class IntakeConstants {
    public static final int feederID = 14;
    public static final int intakeArmID = 34;
    public static final int intakeRollerID = 35;
    public static final double intakePos = -1.28;
    public static final double restPos = -0.75;
    public static final double horizontalPos = 1.43;
    public static final double ground = 1.7;

  }


  public static final class ShooterConstants {
    // flywheel, hood, turret, tower IDs
    public static final int flywheel1ID = 31;
    public static final int flywheel2ID = 0;
    public static final int towerID = 33;

    public static final int flywheelTest = 4000;
  }


  public static final class VisionConstants {
    // array of all connected cameras
    public static final String[] cameraNames = {"Turret"};

    public static final Transform3d[] cameraPositions = {
      new Transform3d(new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(18.75)), new Rotation3d(0, 0, 0)), // turret cam
    };

    public static final int MIN_TAGS = 2;
    public static final double MAX_AMBIGUITY = 0.2;
    public static final double MAX_DISTANCE_METERS = 5.0;
    public static final double BASE_XY_STD_DEV = 0.05;
    public static final double BASE_THETA_STD_DEV = 0.1;
    public static final double MAX_POSE_JUMP_METERS = 1.5;
  }
}
