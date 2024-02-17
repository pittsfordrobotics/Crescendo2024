// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DisabledInstantCommand;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.NewPrettyCommands.AmpCommand;
import frc.robot.commands.NewPrettyCommands.IntakeCommand;
import frc.robot.commands.NewPrettyCommands.PODIUMCommand;
import frc.robot.commands.NewPrettyCommands.SUBWOOFCommand;
import frc.robot.commands.NewPrettyCommands.StoredCommand;
import frc.robot.lib.FFCalculator;
import frc.robot.lib.StructureStates;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

import frc.robot.subsystems.Climber;
import java.io.File;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem;
  private final SendableChooser<Command> toggleDriveMode;
  private final Climber CLIMBER = new Climber();
  private final Shooter SHOOTER = new Shooter();
  private final Intake INTAKE = new Intake();

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(
      OperatorConstants.kOperatorControllerPort);

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    // FF Calculator
    FFCalculator c = FFCalculator.getInstance();
    c.updateIntakePivotAngle(INTAKE::getIntakePivotAngle_deg);
    c.updateShooterAngle(SHOOTER::getShooterAngle_deg);

    // Swerve Stuff
    swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/maxSwerve"));
    toggleDriveMode = new SendableChooser<>();

    Command enhancedHeadingSteeringCommand = swerveSubsystem.driveCommand(
        () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), 0.1),
        () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), 0.1),
        () -> -m_driverController.getRightY(),
        () -> -m_driverController.getRightX(),
        m_driverController::getLeftTriggerAxis,
        m_driverController::getRightTriggerAxis);
    enhancedHeadingSteeringCommand.setName("Enhanced Heading Steer");
    toggleDriveMode.setDefaultOption("Enhanced Steering (BETA)", enhancedHeadingSteeringCommand);

    Command headingSteeringCommand = swerveSubsystem.driveCommand(
        () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), 0.1),
        () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), 0.1),
        () -> -m_driverController.getRightX(),
        () -> -m_driverController.getRightY());
    headingSteeringCommand.setName("Heading Steer");
    toggleDriveMode.addOption("Heading Steering", headingSteeringCommand);

    Command rotationRateSteeringCommand = swerveSubsystem.driveCommand(
        () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), 0.1),
        () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), 0.1),
        () -> -MathUtil.applyDeadband(m_driverController.getRightX(), 0.1));
    rotationRateSteeringCommand.setName("Rotation Rate Steer");
    toggleDriveMode.addOption("Rotation Rate Steering", rotationRateSteeringCommand);

    Shuffleboard.getTab("CONFIG").add(toggleDriveMode);

    DisabledInstantCommand zeroOffsetCommand = new DisabledInstantCommand(swerveSubsystem::setSwerveOffsets);
    zeroOffsetCommand.setName("Zero Offsets");

    Shuffleboard.getTab("CONFIG").add("Zero Swerve Module Offsets", zeroOffsetCommand);

    Shuffleboard.getTab("CONFIG").addString("Current Structure State",
        () -> StructureStates.getCurrentState().toString());

    // Configure the trigger bindings
    // configure_COMP_Bindings();
    configure_TEST_Bindings();
  }

  private void configure_COMP_Bindings() {
    // Swerve
    m_driverController.start().onTrue(new DisabledInstantCommand(() -> swerveSubsystem.zeroGyro()));
    m_driverController.leftBumper().onTrue(swerveSubsystem.setSlowSpeed()).onFalse(swerveSubsystem.setNormalSpeed());
    Command driveCommand = toggleDriveMode.getSelected();
    swerveSubsystem.setDefaultCommand(driveCommand);

    // // states
    StoredCommand storedCommand = new StoredCommand(SHOOTER, INTAKE);
    IntakeCommand intakeCommand = new IntakeCommand(SHOOTER, INTAKE);
    AmpCommand ampCommand = new AmpCommand(SHOOTER, INTAKE);
    SUBWOOFCommand subwoofCommand = new SUBWOOFCommand(SHOOTER, INTAKE);
    PODIUMCommand podiumCommand = new PODIUMCommand(SHOOTER, INTAKE);

    m_operatorController.a().onTrue(ampCommand); // Untested
    m_operatorController.b().onTrue(subwoofCommand); // Untested
    m_operatorController.y().onTrue(podiumCommand); // Untested

    // m_driverController.x().onTrue(new ConditionalCommand(intakeCommand,
    // storedCommand,
    // () -> StructureStates.getCurrentState() !=
    // StructureStates.structureState.intake));
    m_driverController.x().onTrue(intakeCommand); // Untested
    m_driverController.y().onTrue(storedCommand); // Untested

    // Runs the indexer while the right bumper is held -- essentally a shoot command
    m_driverController.rightBumper().whileTrue(SHOOTER.setIndexer(RobotConstants.INDEXER_SHOOT_SPEED))
        .whileFalse(SHOOTER.setIndexer(RobotConstants.INDEXER_IDLE_SPEED));

    // Climber toggle on rightbumper
    m_operatorController.rightBumper().onTrue(CLIMBER.extend());
    m_operatorController.rightBumper().onFalse(CLIMBER.retract());
  }

  private void configure_TEST_Bindings() {
    // Swerve Drive Command chooser
    m_driverController.start().onTrue(new DisabledInstantCommand(() -> swerveSubsystem.zeroGyro()));
    m_driverController.leftBumper().onTrue(swerveSubsystem.setSlowSpeed()).onFalse(swerveSubsystem.setNormalSpeed());

    Command driveCommand = toggleDriveMode.getSelected();
    swerveSubsystem.setDefaultCommand(driveCommand);
    toggleDriveMode.onChange(command -> {
      Command currentDefault = swerveSubsystem.getDefaultCommand();
      swerveSubsystem.removeDefaultCommand();
      CommandScheduler.getInstance().cancel(currentDefault);
      swerveSubsystem.setDefaultCommand(command);
      System.out.println(swerveSubsystem.getDefaultCommand().getName());
    });

    // Remember zeroed at intake pose
    // +RPM means note goes out & +Angle means move up relative to intake pose
    SmartDashboard.putNumber("ShooterPivotAngle_CHANGEME", 0);
    SmartDashboard.putNumber("ShooterRPM_CHANGEME", 0);
    SmartDashboard.putNumber("IntakePivotAngle_CHANGEME", 180);

    // LEFT BUMPER & TRIGGER -> shooter pivot -- Works (tune pids and FF tho)
    m_operatorController.leftBumper().onTrue(SHOOTER.setShooterPivotangle(53));
    m_operatorController.leftBumper().onFalse(SHOOTER.setShooterPivotangle(0.0));
    m_operatorController.leftTrigger().whileTrue(SHOOTER.setShooterPivotangleSupplier());

    // RIGHT BUMPER & TRIGGER -> intake pivot -- Works (tune pids and FF tho)
    m_operatorController.rightBumper().onFalse(INTAKE.setIntakePivotAngle(180));
    m_operatorController.rightBumper().onTrue(INTAKE.setIntakePivotAngle(0));
    m_operatorController.rightTrigger().whileTrue(INTAKE.setIntakePivotAngleSupplier());

    // A -> Shooter RPM (X for supplier) -- Works
    m_operatorController.a().onTrue(SHOOTER.setshooterRPM(5400));
    m_operatorController.a().onFalse(SHOOTER.setshooterRPM(-1000));
    m_operatorController.x().whileTrue(SHOOTER.setshooterRPMSupplier());

    // B -> Intake RAW command -- Untested
    m_operatorController.b().onTrue(INTAKE.setIntakeRpmRAW(0.7));
    m_operatorController.b().onFalse(INTAKE.setIntakeRpmRAW(0));

    // Y -> Indexer test -- Works
    m_operatorController.y().onTrue(SHOOTER.setIndexer(0.5));
    m_operatorController.y().onFalse(SHOOTER.setIndexer(-0.1));
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
