// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DisabledInstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.NewPrettyCommands.AmpCommand;
import frc.robot.commands.NewPrettyCommands.IntakeCommand;
import frc.robot.commands.NewPrettyCommands.SpeakerCommand;
import frc.robot.commands.NewPrettyCommands.StoredCommand;
import frc.robot.lib.FFCalculator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Climber;
import java.io.File;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem;
  // private final Climber CLIMBER = new Climber();
  // private final Shooter SHOOTER = new Shooter();
  // private final Intake INTAKE = new Intake();

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(
      OperatorConstants.kOperatorControllerPort);

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    FFCalculator c = FFCalculator.getInstance();
    // c.updateIntakePivotAngle(INTAKE::getIntakePivotAngle_deg);
    // c.updateShooterAngle(SHOOTER::getShooterAngle_deg);
    swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/maxSwerve"));
    DisabledInstantCommand zeroOffsetCommand = new DisabledInstantCommand(swerveSubsystem::setSwerveOffsets);
    zeroOffsetCommand.setName("Zero Offsets");
    Shuffleboard.getTab("CONFIG").add("Zero Swerve Module Offsets", zeroOffsetCommand);
    // Configure the trigger bindings
    configure_COMP_Bindings();
    // configure_TEST_Bindings();
  }

  private void configure_COMP_Bindings() {
    // Swerve Drive Command
    // This command works for sim, there is no need for a separate sim drive command
    // The sim drive command's angle is position-based and not commanded by angular
    // velocity, so this should be used regardless
    Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveCommand(
        () -> -1 * applyDeadband(m_driverController.getLeftY(), 0.2),
        () -> -1 * applyDeadband(m_driverController.getLeftX(), 0.2),
        () -> -1 * applyDeadband(m_driverController.getRightX(), 0.2));
    swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    m_driverController.start().onTrue(new InstantCommand(() -> {swerveSubsystem.zeroGyro();System.out.println("Resetting gyro");}));

    // // states
    // AmpCommand ampCommand = new AmpCommand(SHOOTER, INTAKE);
    // m_driverController.a().onTrue(ampCommand);

    // SpeakerCommand speakerCommand = new SpeakerCommand(SHOOTER, INTAKE);
    // m_driverController.b().onTrue(speakerCommand);

    // IntakeCommand intakeCommand = new IntakeCommand(SHOOTER, INTAKE);
    // m_driverController.x().onTrue(intakeCommand);

    // StoredCommand storedCommand = new StoredCommand(SHOOTER, INTAKE);
    // m_driverController.y().onFalse(storedCommand);

    // // Runs the indexer while the right bumper is held -- essentally a shootcommand
    // m_driverController.rightBumper().whileTrue(SHOOTER.setIndexer(RobotConstants.INDEXER_SHOOT_SPEED));

    // // Climber toggle on x
    // m_operatorController.x().toggleOnTrue(CLIMBER.extend());
    // m_operatorController.x().toggleOnFalse(CLIMBER.retract());
  }

  private void configure_TEST_Bindings() {
    // Remember zeroed at intake pose -- +RPM means note goes out -- +Angle means move up relative to intake pose
    // left bumper for shooter pivot pid test -- Untested -- make sure to get some sorta rate saftey net
    // m_operatorController.leftBumper().onTrue(SHOOTER.setShooterPivotangle(60));
    // m_operatorController.leftBumper().onTrue(SHOOTER.setShooterPivotraw(0.05));

    // right bumper for intake pivot pid test -- Untested -- make sure to get some sorta rate saftey net
    // m_operatorController.rightBumper().onTrue(INTAKE.setIntakePivotAngle(90));
    // m_operatorController.rightBumper().onTrue(INTAKE.intakePivotRaw(.05));


    // a button for shooter rpm pid test times 2.5 bc it works \-.-/ "theory only
    // gets you so far" - Tested and works
    // m_operatorController.a().onTrue(SHOOTER.setshooterRPM(5700*2.5));
    // m_operatorController.a().onFalse(SHOOTER.setshooterRPM(-500*2.5));

    // // b button for intake test -- Untested
    // m_operatorController.b().onTrue(INTAKE.setIntakeRpmRAW(0.7));
    // m_operatorController.b().onFalse(INTAKE.setIntakeRpmRAW(0));

    // // y button for indexer test -- Tested and works
    // m_operatorController.y().onTrue(SHOOTER.setIndexer(0.5));
    // m_operatorController.y().onFalse(SHOOTER.setIndexer(-0.1));
  }

  // Use this to pass the autonomous command to the main {@link Robot} class.
  // @return the command to run in autonomous
  public Command getAutonomousCommand() {
    // // Choreo swerve auto
    return null;
    // return Autos.choreoSwerveAuto(m_swerveDrive, "NewPath");
  }

  private double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      return value;
    }
    return 0;
  }
}
