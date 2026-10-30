// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

/**
 * Represents a full Swerve Module.
 * @author Koltin Scane
 */
public class SwerveModuleSubsystem extends SubsystemBase {
  private static final CANBus kCANBus = new CANBus("CANivore");

  // objects and variables instantiation
  public TalonFX m_propulsionMotor, m_turningMotor;
  public CANcoder m_encoder;
  public Angle m_encoderOffset;
  public boolean m_encoderReversed;
  private double lastAngleRotations = 0.0;

  // objects for open loop propulsion velocity control
  public PIDController propulsionPID = new PIDController(SwerveConstants.propulsionPIDkp, SwerveConstants.propulsionPIDki, SwerveConstants.propulsionPIDkd);
  public SimpleMotorFeedforward propulsionFF = new SimpleMotorFeedforward(SwerveConstants.propulsionFFks, SwerveConstants.propulsionFFkv, SwerveConstants.propulsionFFka);


  public SwerveModuleSubsystem(int propulsionMotorID, int turningMotorID, int encoderID, Angle encoderOffset,
      boolean encoderReversed, InvertedValue driveInverted, InvertedValue turningInverted) {
    m_propulsionMotor = new TalonFX(propulsionMotorID, kCANBus);
    m_turningMotor = new TalonFX(turningMotorID, kCANBus);

    // defines configs for the turning motors
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.StatorCurrentLimit = 100;
    driveConfig.CurrentLimits.SupplyCurrentLimit = 40;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput.Inverted = driveInverted;
    driveConfig.Voltage.PeakForwardVoltage = 12.0;
    driveConfig.Voltage.PeakReverseVoltage = -12.0;

    // defines configs for the turning motors
    TalonFXConfiguration turningConfig = new TalonFXConfiguration();
    turningConfig.CurrentLimits.StatorCurrentLimit = 50;
    turningConfig.CurrentLimits.SupplyCurrentLimit = 70;
    turningConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turningConfig.MotorOutput.Inverted = turningInverted;
    turningConfig.Voltage.PeakForwardVoltage = 12.0;
    turningConfig.Voltage.PeakReverseVoltage = -12.0;
    turningConfig.Feedback.FeedbackRemoteSensorID = encoderID;
    turningConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    turningConfig.Slot0.kP = 60;
    turningConfig.Slot0.kI = 0.0;
    turningConfig.Slot0.kD = 0.5;
    turningConfig.Slot0.kS = 0.35;
    turningConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.05;
    turningConfig.Feedback.RotorToSensorRatio = SwerveConstants.kSteerGearRatio;

    // applies the configs to the motors
    m_propulsionMotor.getConfigurator().apply(new TalonFXConfiguration());
    m_propulsionMotor.getConfigurator().apply(driveConfig);
    m_turningMotor.getConfigurator().apply(new TalonFXConfiguration());
    m_turningMotor.getConfigurator().apply(turningConfig);

    // makes a new absolute encoder, get the
    m_encoder = new CANcoder(encoderID, kCANBus);
    m_encoderOffset = encoderOffset;
    m_encoderReversed = encoderReversed;
    
    // syncs the turn motor encoder to the CanCoder
    resetToAbsolute();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("getName()", true);
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("SwerveModule [" + m_encoder.getDeviceID() + "]", getCANCoder().getRotations());
  }

  /**
   * Resets the turning motor's encoder to the absolute position of the CanCoder.
   */
  public void resetToAbsolute() {
    double absoluteRotations =
        (m_encoder.getAbsolutePosition().getValueAsDouble()
        * (m_encoderReversed ? -1 : 1));

    m_turningMotor.setPosition(absoluteRotations);
    lastAngleRotations = absoluteRotations;
  }

  public double getPropulsionPosition() {
    // returns the position of the propulsion motor in meters
    return ((m_propulsionMotor.getPosition().getValueAsDouble()) / Constants.SwerveConstants.kDriveGearRatio) * (Constants.SwerveConstants.wheelRadiusMeters * 2 * Math.PI);
  }

  public double getPropulsionVelocity() {
    // returns the velocity of the propulsion motor in meters per second
    return ((m_propulsionMotor.getVelocity().getValueAsDouble()) / Constants.SwerveConstants.kDriveGearRatio) * (Constants.SwerveConstants.wheelRadiusMeters * 2 * Math.PI);
  }

  /**
   * @return Absolute angle of the module relative to the CanCoder.
   */
  public Rotation2d getCANCoder() {
    // returns the position of the absolute encoder in radians
    return Rotation2d.fromRotations((m_encoder.getAbsolutePosition().getValueAsDouble()) * (m_encoderReversed ? -1 : 1));
  }

  /**
   * @return Positon of the turn motor.
   */
  public Rotation2d getTurnPosition() {
    return Rotation2d.fromRotations((m_turningMotor.getPosition().getValueAsDouble()));
  }

  /**
   * Gets a module state (speed and rotation).
   * @return Current swerve module state.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getPropulsionVelocity(), getCANCoder());
  }

  /**
   * Gets a module position (distance traveled and rotation).
   * @return Current swerve module position.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPropulsionPosition(), getCANCoder());
  }

  /**
   * 
   * @param state The desired state of the swerve module.
   * @param omegaRadPerSec 
   */
  public void setState(SwerveModuleState state, double omegaRadPerSec) {
    boolean stopped = Math.abs(state.speedMetersPerSecond) < 0.05 && Math.abs(omegaRadPerSec) < 0.05;

    if (stopped) {
      m_turningMotor.setControl(new PositionVoltage(lastAngleRotations));
      m_propulsionMotor.setVoltage(0);
      return;
    }

    double currentRot = m_turningMotor.getPosition().getValueAsDouble();
    double targetRot  = state.angle.getRotations();
    double delta = targetRot - currentRot;
    delta = MathUtil.inputModulus(delta, -0.5, 0.5);

    if (Math.abs(delta) > 0.25) {
      delta -= Math.signum(delta) * 0.5;
      state = new SwerveModuleState(-state.speedMetersPerSecond, state.angle.plus(Rotation2d.fromRotations(0.5)));
    }
    
    double unwrappedTarget = currentRot + delta;
    lastAngleRotations = unwrappedTarget;

    double angleError = unwrappedTarget - currentRot;

    if (Math.abs(angleError) < 0.002) {
      m_turningMotor.stopMotor();
    } else {
      m_turningMotor.setControl(new PositionVoltage(unwrappedTarget));
    }

    double speedScale = Math.cos(angleError * 2.0 * Math.PI);
    double driveSpeed = state.speedMetersPerSecond * speedScale;

    propulsionPID.setSetpoint(driveSpeed);

    m_propulsionMotor.setVoltage(propulsionPID.calculate(getPropulsionVelocity()) + propulsionFF.calculate(driveSpeed));
  }
}
