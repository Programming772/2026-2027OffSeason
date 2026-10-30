package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.SwerveConstants;

/** Holds a gyro-relative heading when the driver releases the rotation input. */
public final class HeadingController {
  private final PIDController pid = new PIDController(
      SwerveConstants.thetaPIDkp, SwerveConstants.thetaPIDki, SwerveConstants.thetaPIDkd);
  private Rotation2d target;
  private boolean locked = true;

  public HeadingController(Rotation2d initialHeading) {
    target = initialHeading;
    pid.enableContinuousInput(-Math.PI, Math.PI);
  }

  public double calculate(Rotation2d heading, double requestedOmega) {
    if (Math.abs(requestedOmega) < 0.05) {
      if (!locked) {
        setTarget(heading);
      }
      requestedOmega = pid.calculate(heading.getRadians(), target.getRadians());
    } else {
      locked = false;
    }
    return MathUtil.clamp(requestedOmega, -4, 4);
  }

  public void setTarget(Rotation2d heading) {
    target = heading;
    pid.reset();
    locked = true;
  }
}
