// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.HeadingController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;

/**
 * Represents a full Swerve Drive.
 * @author Koltin Scane
 */
public class SwerveDriveSubsystem extends SubsystemBase {
  private final SwerveModuleSubsystem [] modules = {
    // makes a SwerveModuleSubsystem for the front left module
    new SwerveModuleSubsystem(
      Constants.SwerveConstants.FrontLeftConstants.frontLeftPropulsionID, 
      Constants.SwerveConstants.FrontLeftConstants.frontLeftTurningID, 
      Constants.SwerveConstants.FrontLeftConstants.frontLeftCANCoderID, 
      Constants.SwerveConstants.FrontLeftConstants.frontLeftCANCoderOffset, 
      false,
      InvertedValue.Clockwise_Positive,
      InvertedValue.CounterClockwise_Positive
    ),
    // makes a SwerveModuleSubsystem for the front right module
    new SwerveModuleSubsystem(
      Constants.SwerveConstants.FrontRightConstants.frontRightPropulsionID, 
      Constants.SwerveConstants.FrontRightConstants.frontRightTurningID, 
      Constants.SwerveConstants.FrontRightConstants.frontRightCANCoderID, 
      Constants.SwerveConstants.FrontRightConstants.frontRightCANCoderOffset, 
      false,
      InvertedValue.CounterClockwise_Positive,
      InvertedValue.CounterClockwise_Positive
    ),
    // makes a SwerveModuleSubsystem for the back left module
    new SwerveModuleSubsystem(
      Constants.SwerveConstants.BackLeftConstants.backLeftPropulsionID, 
      Constants.SwerveConstants.BackLeftConstants.backLeftTurningID, 
      Constants.SwerveConstants.BackLeftConstants.backLeftCANCoderID, 
      Constants.SwerveConstants.BackLeftConstants.backLeftCANCoderOffset, 
      false,
      InvertedValue.Clockwise_Positive,
      InvertedValue.CounterClockwise_Positive
    ),
    // makes a SwerveModuleSubsystem for the back right module
    new SwerveModuleSubsystem(
      Constants.SwerveConstants.BackRightConstants.backRightPropulsionID, 
      Constants.SwerveConstants.BackRightConstants.backRightTurningID, 
      Constants.SwerveConstants.BackRightConstants.backRightCANCoderID, 
      Constants.SwerveConstants.BackRightConstants.backRightCANCoderOffset, 
      false,
      InvertedValue.CounterClockwise_Positive,
      InvertedValue.CounterClockwise_Positive
    )
  };

  public final Pigeon2 gyro = new Pigeon2(SwerveConstants.pigeonID);
  private final HeadingController headingController = new HeadingController(getRotation());
  private Field2d field = new Field2d();
  private boolean slowDown = false;
  private VisionSubsystem vision = new VisionSubsystem();
  
  // creates a pose estimator object to get the position of the robot relative to the field
  public final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
    Constants.SwerveConstants.driveKinematics,
    getRotation(),
    new SwerveModulePosition[] {
      modules[0].getPosition(),
      modules[1].getPosition(),
      modules[2].getPosition(),
      modules[3].getPosition()
    },
    new Pose2d(),
    VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
    VecBuilder.fill(0.7, 0.7, Units.degreesToRadians(10))
  );

  public SwerveDriveSubsystem() {
    SmartDashboard.putData("Field", field);

    gyro.optimizeBusUtilization(100, 10);
  }
  
  @Override
  public void periodic() {
    // pose estimation relative to robot sensors and vision
    updateOdometryRobotRelative();
    addVisionMeasurements();

    // updates the minifield on Shuffle board to the estimated position of the bot
    field.setRobotPose(poseEstimator.getEstimatedPosition());

    // updates position of simulated robot on ShuffleBoard relative to the odometry
    SmartDashboard.putNumber("Pose X", poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Pose Y", poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Rotation", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
  }
  
  /**
   * @return Rotation of the gyro.
   */
  public Rotation2d getRotation() {
    // returns 
    return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
  }

  /**
   * Sets the yaw of the gyro to 0.
   */
  public void resetHeading() {
    gyro.reset();
    setTargetRot(Rotation2d.kZero);
  }

  /**
   * Updates the swerve drive odometry relative to the onboard sensors.
   */
  public void updateOdometryRobotRelative() {
    poseEstimator.update(
      getRotation(),
      new SwerveModulePosition[] {
        modules[0].getPosition(),
        modules[1].getPosition(),
        modules[2].getPosition(),
        modules[3].getPosition()
      }
    );
  }

  /**
   * Sets the position of swerve drive pose estimator to the supplied pose.
   * @param newPose The new pose of the swerve drive on the field.
   */
  public void autoPose(Pose2d newPose) {
    poseEstimator.resetPosition(
      getRotation(),
      new SwerveModulePosition[] {
        modules[0].getPosition(),
        modules[1].getPosition(),
        modules[2].getPosition(),
        modules[3].getPosition()
      },
      newPose
    );
  }

  /**
   * Gets the pose provided by the swerve drive pose estimator.
   * @return The estimated pose.
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }
  
  /**
   * Gets the swerve drive's speed.
   * @return The chassis speeds.
   */
  public ChassisSpeeds getRobotSpeeds() {
    return Constants.SwerveConstants.driveKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Gets the states of all four swerve drive modules.
   * @return The array of module states.
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      modules[0].getState(),
      modules[1].getState(),
      modules[2].getState(),
      modules[3].getState()
    };
  }

  /**
   * Generates chassis speeds relative to the inputted speeds and applies them to the swerve drive.
   * @param xSpeed The x speed component of the chassis speeds.
   * @param ySpeed The y speed component of the chassis speeds.
   * @param thetaSpeed The rotation speed component of the chassis speeds.
   */
  public void generateSpeeds(double xSpeed, double ySpeed, double thetaSpeed) {
    thetaSpeed = headingController.calculate(getRotation(), thetaSpeed);

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed, getRotation());
    ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    
    setSpeeds(discretizedSpeeds);
  }

  /**
   * Sets the states of each swerve module relative to the given chassis speeds while also applying an optional slow speed. 
   * @param speeds The target chassis speeds of the swerve drive.
   */
  public void setSpeeds(ChassisSpeeds speeds) {
    SwerveModuleState[] states =
      Constants.SwerveConstants.driveKinematics
        .toSwerveModuleStates(speeds);

    if (slowDown) {
      SwerveDriveKinematics.desaturateWheelSpeeds(
        states, Constants.SwerveConstants.maxVelocity * 0.3
      );
    } else {
      SwerveDriveKinematics.desaturateWheelSpeeds(
        states, Constants.SwerveConstants.maxVelocity
      );
    }

    for (int i = 0; i < 4; i++) {
      modules[i].setState(states[i], speeds.omegaRadiansPerSecond);
    }
  }

  /**
   * Toggles slowed down swerve drive speeds to true.
   */
  public void slowSpeed() {
    slowDown = true;
  }

  /**
   * Toggles slowed down swerve drive speeds to false.
   */
  public void regularSpeed() {
    slowDown = false;
  }

  /**
   * Sets the setpoint of the swerve drive rotation to the given angle.
   * @param angle Target swerve drive angle.
   */
  public void setTargetRot(Rotation2d angle) {
    headingController.setTarget(angle);
  }

  /**
   * Finds the closest 45 degree angle for the swerve drive to hold for safe bumb mobility. 
   */
  public void humpRot() {
    double currentAngle = getRotation().getDegrees();

    setTargetRot(Rotation2d.fromDegrees(45 + 90 * Math.floor(currentAngle / 90)));
  }

  private void addVisionMeasurements() {
    for (var measurement : vision.getVisionMeasurements()) {
      var visionPose = measurement.pose.estimatedPose.toPose2d();
      double timestamp = measurement.pose.timestampSeconds;
      double distance = measurement.avgTagDistance;
      int tagCount = measurement.tagCount;
      var currentPose = poseEstimator.getEstimatedPosition();

      if (visionPose.getTranslation().getDistance(currentPose.getTranslation()) > VisionConstants.MAX_POSE_JUMP_METERS)
        continue;

      double xyStdDev = VisionConstants.BASE_XY_STD_DEV;
      double thetaStdDev = VisionConstants.BASE_THETA_STD_DEV;

      xyStdDev *= distance;
      thetaStdDev *= distance;

      if (tagCount > 1) {
        xyStdDev /= tagCount;
        thetaStdDev /= tagCount;
      }

      poseEstimator.addVisionMeasurement(
        visionPose,
        timestamp,
        VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)
      );
    }
  }

  public void rotateToHub () {
    Pose2d robotPose = poseEstimator.getEstimatedPosition();
    Rotation2d yaw = PhotonUtils.getYawToPose(robotPose, Constants.hubCoords);

    // The heading controller uses gyro coordinates; vision and auto pose resets
    // can give the field pose a different heading offset.
    setTargetRot(getRotation().plus(yaw).plus(Rotation2d.k180deg));
  }
}
