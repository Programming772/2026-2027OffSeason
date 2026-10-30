package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class HeadingControllerTest {
  @Test
  void correctsTowardTargetInBothDirections() {
    HeadingController controller = new HeadingController(Rotation2d.kZero);
    assertTrue(controller.calculate(Rotation2d.fromDegrees(-10), 0) > 0);
    assertTrue(controller.calculate(Rotation2d.fromDegrees(10), 0) < 0);
  }

  @Test
  void takesShortestTurnAcrossAngleBoundary() {
    HeadingController controller = new HeadingController(Rotation2d.fromDegrees(-179));
    double output = controller.calculate(Rotation2d.fromDegrees(179), 0);
    assertEquals(Constants.SwerveConstants.thetaPIDkp * Math.toRadians(2), output, 1e-9);
  }

  @Test
  void releaseCapturesCurrentHeading() {
    HeadingController controller = new HeadingController(Rotation2d.kZero);
    assertEquals(1, controller.calculate(Rotation2d.fromDegrees(20), 1));
    assertEquals(0, controller.calculate(Rotation2d.fromDegrees(30), 0), 1e-9);
    assertTrue(controller.calculate(Rotation2d.fromDegrees(35), 0) < 0);
  }

  @Test
  void explicitTargetSurvivesReleaseOfManualRotation() {
    HeadingController controller = new HeadingController(Rotation2d.kZero);
    controller.calculate(Rotation2d.fromDegrees(20), 1);
    controller.setTarget(Rotation2d.fromDegrees(45));
    assertTrue(controller.calculate(Rotation2d.fromDegrees(20), 0) > 0);
  }

  @Test
  void resetTargetDoesNotReturnToOldHeading() {
    HeadingController controller = new HeadingController(Rotation2d.fromDegrees(90));
    controller.setTarget(Rotation2d.kZero);
    assertEquals(0, controller.calculate(Rotation2d.kZero, 0), 1e-9);
  }

  @Test
  void limitsBothDriverAndCorrectionSpeeds() {
    HeadingController controller = new HeadingController(Rotation2d.kZero);
    assertEquals(4, controller.calculate(Rotation2d.kZero, 10));
    assertEquals(-4, controller.calculate(Rotation2d.kZero, -10));
    controller.setTarget(Rotation2d.fromDegrees(170));
    assertEquals(4, controller.calculate(Rotation2d.kZero, 0));
  }
}
