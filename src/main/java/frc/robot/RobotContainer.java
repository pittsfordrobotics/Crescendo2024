// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.DisabledInstantCommand;
import frc.robot.commands.NewPrettyCommands.AmpCommand;
import frc.robot.commands.NewPrettyCommands.IntakeCommand;
import frc.robot.commands.NewPrettyCommands.SpeakerCommand;
import frc.robot.commands.NewPrettyCommands.StoredCommand;
import frc.robot.lib.FFCalculator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem;
  private final SendableChooser<Command> driveModeChooser;
  private final Climber climber;
  private final Shooter shooter;
  private final Intake intake;

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(
      OperatorConstants.kOperatorControllerPort);

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    climber = new Climber();
    shooter = new Shooter();
    intake = new Intake();
    swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/maxSwerve"));
    FFCalculator c = FFCalculator.getInstance();
    // c.updateIntakePivotAngle(intake::getIntakePivotAngle_deg);
    c.updateShooterAngle(shooter::getShooterAngleDeg);
    driveModeChooser = new SendableChooser<>();
    Command enhancedHeadingSteeringCommand = swerveSubsystem.enhancedHeadingDriveCommand(
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightY(),
            () -> -m_driverController.getRightX(),
            m_driverController::getLeftTriggerAxis,
            m_driverController::getRightTriggerAxis);
    enhancedHeadingSteeringCommand.setName("Enhanced Heading Steer");
    Command headingSteeringCommand = swerveSubsystem.headingDriveCommand(
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX(),
            () -> -m_driverController.getRightY());
    headingSteeringCommand.setName("Heading Steer");
    Command rotationRateSteeringCommand = swerveSubsystem.rotationRateDriveCommand(
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX());
    rotationRateSteeringCommand.setName("Rotation Rate Steer");
    driveModeChooser.addOption("Enhanced Steering (BETA)", enhancedHeadingSteeringCommand);
    driveModeChooser.addOption("Heading Steering", headingSteeringCommand);
    driveModeChooser.setDefaultOption("Rotation Rate Steering", rotationRateSteeringCommand);
    Shuffleboard.getTab("CONFIG").add(driveModeChooser);
    DisabledInstantCommand zeroOffsetCommand = new DisabledInstantCommand(swerveSubsystem::setSwerveOffsets);
    zeroOffsetCommand.setName("Zero Offsets");
    Shuffleboard.getTab("CONFIG").add("Zero Swerve Module Offsets", zeroOffsetCommand);
    // Configure the trigger bindings
    configure_COMP_Bindings();
    // configure_TEST_Bindings();
  }

  private void configure_COMP_Bindings() {
    // Swerve
    m_driverController.start().onTrue(new DisabledInstantCommand(swerveSubsystem::zeroGyro));
    m_driverController.leftBumper().onTrue(swerveSubsystem.setSlowSpeed()).onFalse(swerveSubsystem.setNormalSpeed());
    Command driveCommand = driveModeChooser.getSelected();
    swerveSubsystem.setDefaultCommand(driveCommand);
    driveModeChooser.onChange(command -> {
      Command currentDefault = swerveSubsystem.getDefaultCommand();
      swerveSubsystem.removeDefaultCommand();
      CommandScheduler.getInstance().cancel(currentDefault);
      swerveSubsystem.setDefaultCommand(command);
      System.out.println(swerveSubsystem.getDefaultCommand().getName());
    });
    m_driverController.start().onTrue(new InstantCommand(() -> {swerveSubsystem.zeroGyro();System.out.println("Resetting gyro");}));

    // // states
    AmpCommand ampCommand = new AmpCommand(shooter, intake);
    m_operatorController.a().onTrue(ampCommand);

    SpeakerCommand speakerCommand = new SpeakerCommand(shooter, intake);
    m_operatorController.b().onTrue(speakerCommand);

    IntakeCommand intakeCommand = new IntakeCommand(shooter, intake);
    m_operatorController.x().onTrue(intakeCommand);

    StoredCommand storedCommand = new StoredCommand(shooter, intake);
    m_operatorController.y().onTrue(storedCommand);

    // Runs the indexer while the right bumper is held -- essentally a shootcommand
    m_driverController.rightBumper().whileTrue(shooter.spinIndexerCommand(RobotConstants.INDEXER_SHOOT_SPEED)).whileFalse(shooter.spinIndexerCommand(RobotConstants.INDEXER_IDLE_SPEED));

    // Climber toggle on rightbumper
    m_operatorController.rightBumper().onTrue(climber.extendCommand());
    m_operatorController.rightBumper().onFalse(climber.retractCommand());
  }

  private void configure_TEST_Bindings() {
    // Swerve Drive Command chooser
    m_driverController.start().onTrue(new DisabledInstantCommand(swerveSubsystem::zeroGyro));
    m_driverController.leftBumper().onTrue(swerveSubsystem.setSlowSpeed()).onFalse(swerveSubsystem.setNormalSpeed());

    Command driveCommand = driveModeChooser.getSelected();
    swerveSubsystem.setDefaultCommand(driveCommand);
    driveModeChooser.onChange(command -> {
      Command currentDefault = swerveSubsystem.getDefaultCommand();
      swerveSubsystem.removeDefaultCommand();
      CommandScheduler.getInstance().cancel(currentDefault);
      swerveSubsystem.setDefaultCommand(command);
      System.out.println(swerveSubsystem.getDefaultCommand().getName());
    });

    // Remember zeroed at intake pose -- +RPM means note goes out -- +Angle means
    // move up relative to intake pose
    SmartDashboard.putNumber("shooterPivotAngle_CHANGEME", 0);
    SmartDashboard.putNumber("shooterRPM_CHANGEME", 0);
    SmartDashboard.putNumber("IntakePivotAngle_CHANGEME", 180);

    // left bumper for shooter pivot pid test -- Works (tune pids and FF tho)
    m_operatorController.leftBumper().onTrue(shooter.setPivotAngleCommand(53));
    m_operatorController.leftBumper().onFalse(shooter.setPivotAngleCommand(0.0));
    // left trigger for shooter pivot based on suplier -- untested
    m_operatorController.leftTrigger().whileTrue(
        shooter.setPivotAngleSupplier(() -> SmartDashboard.getNumber("shooterPivotAngle_CHANGEME", 0)));

    // right bumper for intake pivot pid test -- Works (tune pids and FF tho)
    m_operatorController.rightBumper().onFalse(intake.setPivotAngleCommand(180));
    m_operatorController.rightBumper().onTrue(intake.setPivotAngleCommand(0));
    // right trigger for intake pivot based on suplier -- untested
    m_operatorController.rightTrigger().whileTrue(
        intake.setPivotAngleSupplier(() -> SmartDashboard.getNumber("IntakePivotAngle_CHANGEME", 170)));

    // shooter PID RPM -- Works
    m_operatorController.a().onTrue(shooter.setShooterRPMCommand(5400));
    m_operatorController.a().onFalse(shooter.setShooterRPMCommand(-1000));
    // shooter RPM based on supplier -- untested
    m_operatorController.x()
        .onTrue(shooter.setShooterRPMSupplier(() -> SmartDashboard.getNumber("shooterRPM_CHANGEME", 0)));
    m_operatorController.x().onFalse(shooter.setShooterRPMCommand(-1000));

    // Intake RAW command -- Untested
    m_operatorController.b().onTrue(intake.spinIntakeCommand(0.7));
    m_operatorController.b().onFalse(intake.spinIntakeCommand(0));

    // Indexer test -- Works
    m_operatorController.y().onTrue(shooter.spinIndexerCommand(0.5));
    m_operatorController.y().onFalse(shooter.spinIndexerCommand(-0.1));
  }

  // Use this to pass the autonomous command to the main {@link Robot} class.
  // @return the command to run in autonomous
  public Command getAutonomousCommand() {
    // // Choreo swerve auto
    return null;
    // return Autos.choreoSwerveAuto(m_swerveDrive, "NewPath");
  }
}
