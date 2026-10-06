// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared.
 */
public class RobotContainer {

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveSubsystem m_driveSubsystem =
    new DriveSubsystem();

  // Xbox controller
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. */
  public RobotContainer() {
    configureBindings();
  }

  /**
   * Configure controller button bindings.
   */
  private void configureBindings() {

    // When the A button is pressed, print a message
    m_driverController.a().whileTrue(
        m_driveSubsystem.run(
            m_driveSubsystem::runMotor
        )
    );

    m_driverController.a().onFalse(
        m_driveSubsystem.runOnce(
            m_driveSubsystem::stopMotor
        )
    );
  }

  /**
   * Use this to pass the autonomous command to the main Robot class.
   */
  public Command getAutonomousCommand() {
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
