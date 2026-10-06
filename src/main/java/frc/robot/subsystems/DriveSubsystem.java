package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  private static final int KRAKEN_CAN_ID = 35;
  private static final double MOTOR_VOLTAGE = 2.0;

  private final TalonFX kraken = new TalonFX(KRAKEN_CAN_ID);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public DriveSubsystem() {
    stopMotor();
  }

  // Run the motor at a constant speed.
  public void runMotor() {
    kraken.setControl(
        voltageRequest.withOutput(MOTOR_VOLTAGE)
    );
  }

  // Stop the motor.
  public void stopMotor() {
    kraken.setControl(
        voltageRequest.withOutput(0.0)
    );
  }

  @Override
  public void periodic() {
  }
}
