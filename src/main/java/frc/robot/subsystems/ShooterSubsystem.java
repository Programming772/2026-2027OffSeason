// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private static final CANBus kCANBus = new CANBus("CANivore");

  TalonFX flywheel1 = new TalonFX(ShooterConstants.flywheel1ID);
  TalonFX flywheel2 = new TalonFX(ShooterConstants.flywheel2ID);
  TalonFX tower = new TalonFX(ShooterConstants.towerID, kCANBus);

  public ShooterSubsystem() {
    //turret.setPosition(0);
    TalonFXConfiguration flywheel1Config = new TalonFXConfiguration();
    flywheel1Config.CurrentLimits.StatorCurrentLimit = 100;
    flywheel1Config.CurrentLimits.SupplyCurrentLimit = 60;
    flywheel1Config.Slot0.kS = 0;
    flywheel1Config.Slot0.kV = 0;
    flywheel1Config.Slot0.kP = 0;
    flywheel1Config.Slot0.kI = 0;
    flywheel1Config.Slot0.kD = 0;
    flywheel1Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheel1Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    flywheel1Config.Voltage.PeakForwardVoltage = 12.0;
    flywheel1Config.Voltage.PeakReverseVoltage = -12.0;

    TalonFXConfiguration flywheel2Config = new TalonFXConfiguration();
    flywheel2Config.CurrentLimits.StatorCurrentLimit = 100;
    flywheel2Config.CurrentLimits.SupplyCurrentLimit = 60;
    flywheel2Config.Slot0.kS = 0;
    flywheel2Config.Slot0.kV = 0;
    flywheel2Config.Slot0.kP = 0;
    flywheel2Config.Slot0.kI = 0;
    flywheel2Config.Slot0.kD = 0;
    flywheel2Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheel2Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    flywheel2Config.Voltage.PeakForwardVoltage = 12.0;
    flywheel2Config.Voltage.PeakReverseVoltage = -12.0;
    
    TalonFXConfiguration towerConfig = new TalonFXConfiguration();
    towerConfig.CurrentLimits.StatorCurrentLimit = 100;
    towerConfig.CurrentLimits.SupplyCurrentLimit = 60;
    towerConfig.Slot0.kS = 0;
    towerConfig.Slot0.kV = 0;
    towerConfig.Slot0.kP = 0;
    towerConfig.Slot0.kI = 0;
    towerConfig.Slot0.kD = 0;
    towerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    towerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    towerConfig.Voltage.PeakForwardVoltage = 12.0;
    towerConfig.Voltage.PeakReverseVoltage = -12.0;

    // applies the configs to the motors
    flywheel1.getConfigurator().apply(new TalonFXConfiguration());
    flywheel1.getConfigurator().apply(flywheel1Config);
    flywheel2.getConfigurator().apply(new TalonFXConfiguration());
    flywheel2.getConfigurator().apply(flywheel2Config);
    tower.getConfigurator().apply(new TalonFXConfiguration());
    tower.getConfigurator().apply(towerConfig);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Flywheel1 RPM", getFlywheel1RPM());
    SmartDashboard.putNumber("Flywheel2 RPM", getFlywheel2RPM());
    SmartDashboard.putNumber("Tower RPM", getTowerRPM());
  }

  public void setflywheel1RPM(double targetRPM) {
    flywheel1.setControl(new VelocityVoltage(targetRPM / 60).withSlot(0));
  }

  public double getFlywheel1RPM() {
    return flywheel1.getVelocity().getValueAsDouble() * 60;
  }

  public void manualFlywheel1(double speed) {
    flywheel1.set(speed);
  }

  public void setflywheel2RPM(double targetRPM) {
    flywheel2.setControl(new VelocityVoltage(targetRPM / 60).withSlot(0));
  }

  public double getFlywheel2RPM() {
    return flywheel2.getVelocity().getValueAsDouble() * 60;
  }

  public void manualFlywheel2(double speed) {
    flywheel2.set(speed);
  }

  public void setTowerRPM(double targetRPM) {
    tower.setControl(new VelocityVoltage(targetRPM / 60).withSlot(0));
  }

  public double getTowerRPM() {
    return tower.getVelocity().getValueAsDouble() * 60;
  }

  public void manualTower(double speed) {
    tower.set(speed);
  }
}
