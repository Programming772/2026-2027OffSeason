// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleConsumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleoperatedControlCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotContainer {
  final DoubleSolenoid climber = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 4);

  final SwerveDriveSubsystem swerveSubsystem = new SwerveDriveSubsystem();
  final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  final Joystick driverController = new Joystick(0);
  final Joystick testController = new Joystick(1);
  final Joystick manualControls = new Joystick(2);

  SendableChooser<Command> autoChooser;

  // driver controls
  //Trigger climbToggle = new JoystickButton(driverController, 1);
  Trigger hold45 = new JoystickButton(driverController, 5);
  Trigger intakeIntake = new JoystickButton(driverController, 1);
  Trigger intakeRest = new JoystickButton(driverController, 4);
  // Trigger slow = new JoystickButton(driverController, 8);
  Trigger resetHeading = new JoystickButton(driverController, 8);
  Trigger feedingToggle = new JoystickButton(driverController, 7);
  // Trigger midShooting = new JoystickButton(driverController, 5);
  Trigger toggleShooting = new JoystickButton(driverController, 6);
  Trigger autoAim = new JoystickButton(driverController, 10);
  Trigger aimAtHub = new JoystickButton(driverController, 2);

  Trigger manualTurretCCW = new JoystickButton(testController, 5);
  Trigger manualTurretCW = new JoystickButton(testController, 6);

  // test controls
  // //Trigger hoodTest = new JoystickButton(testController, 1);
  // Trigger turretTest = new JoystickButton(testController, 1);
  // Trigger turretTest1 = new JoystickButton(testController, 2);
  // //Trigger towerTest = new JoystickButton(testController, 3);
  // Trigger flywheelTest = new JoystickButton(testController, 1);
  // Trigger flywheelTest1 = new JoystickButton(testController, 3);
  // Trigger flywheelTest2 = new JoystickButton(testController, 2);
  
  //Trigger feederTest = new JoystickButton(testController, 7);
  Trigger intake = new JoystickButton(testController, 1);
  //Trigger intakeRest = new JoystickButton(testController, 4);
  //Trigger intakeArmTest = new JoystickButton(testController, 1);
  //Trigger intakeArmTest1 = new JoystickButton(testController, 4);
  //Trigger intakeArmTest2 = new JoystickButton(testController, 2);
  //Trigger intakeRollersTest = new JoystickButton(testController, 6);

  // manual controls
  Trigger slowTower = new JoystickButton(manualControls, 1);
  Trigger slowFlywheel = new JoystickButton(manualControls, 2);
  Trigger slowFeeder = new JoystickButton(manualControls, 3);
  Trigger slowIntakeRollers = new JoystickButton(manualControls, 4);
  Trigger turretCW = new JoystickButton(manualControls, 5);
  Trigger turretCCW = new JoystickButton(manualControls, 6);
  Trigger intakeUp = new JoystickButton(manualControls, 8);
  Trigger intakeDown = new JoystickButton(manualControls, 7);
  Trigger hoodFwd = new JoystickButton(manualControls, 9);
  Trigger hoodRvs = new JoystickButton(manualControls, 10);


  private RobotConfig config;

  public RobotContainer() {
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      DriverStation.reportError("Failed to load PathPlanner config", e.getStackTrace());
    }

    // Keep driver controls available if the deployed autonomous settings are invalid.
    if (config != null) {
      configureAutoBuilder();
      autoChooser = AutoBuilder.buildAutoChooser();
    } else {
      autoChooser = new SendableChooser<>();
      autoChooser.setDefaultOption("Autonomous unavailable", Commands.none());
    }

    climber.set(Value.kForward);

    swerveSubsystem.setDefaultCommand(new TeleoperatedControlCommand(
      swerveSubsystem,
      () -> -driverController.getRawAxis(1),
      () -> -driverController.getRawAxis(0),
      () -> driverController.getRawAxis(4)
    ));

    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
  }

  private void configureAutoBuilder() {
    AutoBuilder.configure(
      swerveSubsystem::getPose,
      swerveSubsystem::autoPose, 
      swerveSubsystem::getRobotSpeeds, 
      swerveSubsystem::setSpeeds, 
      new PPHolonomicDriveController(
        new PIDConstants(
          Constants.SwerveConstants.translationPIDkp,
          Constants.SwerveConstants.translationPIDki,
          Constants.SwerveConstants.translationPIDkd),
        new PIDConstants(
          Constants.SwerveConstants.thetaPIDkp,
          Constants.SwerveConstants.thetaPIDki,
          Constants.SwerveConstants.thetaPIDkd)
      ), 
      config, 
      () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red, 
      swerveSubsystem
    );

  }

  private void configureBindings() {
    // driver controls
    //climbToggle.onTrue(new InstantCommand(() -> climber.toggle()));
    
    // slow.whileTrue(new InstantCommand(() -> swerveSubsystem.slowSpeed()))
    // .onFalse(new InstantCommand(() -> swerveSubsystem.regularSpeed()));
    
    hold45.onTrue(new InstantCommand(() -> swerveSubsystem.humpRot()));
    resetHeading.onTrue(new InstantCommand(() -> swerveSubsystem.resetHeading()));

    intakeIntake.whileTrue(intakePositionCommand(Constants.IntakeConstants.intakePos));
    intakeRest.whileTrue(intakePositionCommand(Constants.IntakeConstants.restPos));

    configureIntakeRollerBindings(
      feedingToggle, slowIntakeRollers, intakeSubsystem::manualIntakeRoller);

    aimAtHub.onTrue(new InstantCommand(() -> swerveSubsystem.rotateToHub()));


    // test controls
    intake.whileTrue(intakePositionCommand(Constants.IntakeConstants.intakePos))
      .onFalse(intakePositionCommand(0)
        .until(() -> MathUtil.isNear(0, intakeSubsystem.getIntakePosition(), 0.05)));
    //intakeRest.whileTrue(new InstantCommand(() -> intakeSubsystem.moveIntake(Constants.IntakeConstants.restPos))).onFalse(new InstantCommand(() -> intakeSubsystem.moveIntake(0)));
    
    //intakeArmTest2.toggleOnTrue(new InstantCommand(() -> intakeSubsystem.setIntakeArmPosition(Constants.IntakeConstants.ground)));
    //intakeArmTest.toggleOnTrue(new InstantCommand(() -> intakeSubsystem.setIntakeArmPosition(Constants.IntakeConstants.intakePos)));
    //intakeArmTest1.toggleOnTrue(new InstantCommand(() -> intakeSubsystem.setIntakeArmPosition(Constants.IntakeConstants.restPos)));
    
    //intakeRollersTest.toggleOnTrue(new InstantCommand(() -> intakeSubsystem.setIntakeRollerRPM(6000))).toggleOnFalse(new InstantCommand(() -> intakeSubsystem.setIntakeRollerRPM(0)));

    // manual controls
    intakeUp.whileTrue(intakeSubsystem.runEnd(
      () -> intakeSubsystem.manualIntakePosition(0.2),
      () -> intakeSubsystem.manualIntakePosition(0)));
    intakeDown.whileTrue(intakeSubsystem.runEnd(
      () -> intakeSubsystem.manualIntakePosition(-0.2),
      () -> intakeSubsystem.manualIntakePosition(0)));
    }

  private Command intakePositionCommand(double target) {
    return intakeSubsystem.runEnd(
      () -> intakeSubsystem.moveIntake(target),
      () -> intakeSubsystem.manualIntakePosition(0));
  }

  static void configureIntakeRollerBindings(
      Trigger driverRequest, Trigger manualRequest, DoubleConsumer setOutput) {
    // One combined request prevents either controller from stopping the other.
    // No arm requirement: the roller must run alongside intake positioning.
    driverRequest.or(manualRequest).and(DriverStation::isTeleopEnabled)
      .whileTrue(Commands.runEnd(() -> setOutput.accept(0.4), () -> setOutput.accept(0)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
