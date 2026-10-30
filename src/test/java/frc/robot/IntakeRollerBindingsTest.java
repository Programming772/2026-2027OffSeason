package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class IntakeRollerBindingsTest {
  private final CommandScheduler scheduler = CommandScheduler.getInstance();
  private boolean driverPressed;
  private boolean manualPressed;
  private double output;

  @BeforeEach
  void setUp() {
    HAL.initialize(500, 0);
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    scheduler.getDefaultButtonLoop().clear();
    RobotContainer.configureIntakeRollerBindings(
        new Trigger(() -> driverPressed), new Trigger(() -> manualPressed),
        value -> output = value);
  }

  @AfterEach
  void tearDown() {
    scheduler.cancelAll();
    scheduler.getDefaultButtonLoop().clear();
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
  }

  @Test
  void releasingOneControllerDoesNotStopTheOther() {
    driverPressed = true;
    scheduler.run();
    assertEquals(0.4, output);
    manualPressed = true;
    scheduler.run();
    driverPressed = false;
    scheduler.run();
    assertEquals(0.4, output);
    manualPressed = false;
    scheduler.run();
    assertEquals(0, output);
  }

  @Test
  void disableStopsMotorAndReleaseWhileDisabledDoesNotLatchOutput() {
    driverPressed = true;
    scheduler.run();
    assertEquals(0.4, output);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    scheduler.run();
    assertEquals(0, output);
    driverPressed = false;
    scheduler.run();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    scheduler.run();
    assertEquals(0, output);
  }

  @Test
  void heldButtonOnlyRunsWhenTeleopIsEnabled() {
    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();
    driverPressed = true;
    scheduler.run();
    assertEquals(0, output);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();
    scheduler.run();
    assertEquals(0.4, output);
  }
}
