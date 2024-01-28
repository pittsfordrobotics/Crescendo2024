// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.BasicIntake;
import frc.robot.commands.BasicShoot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.Intake;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Commands;

import java.io.File;
import java.util.function.DoubleSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem;
  private final Shooter SHOOTER = new Shooter();
  private final Intake INTAKE = new Intake();

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(
      OperatorConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/maxSwerve"));
    // Configure the trigger bindings
    configureBindings();
  }

  private boolean isFieldOriented() {
    return fieldOrientedButton.getEntry().getBoolean(false);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // This command works for sim, there is no need for a separate sim drive command
    // The sim drive command's angle is position-based and not commanded by angular velocity, so this should be used regardless
    Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveCommand(
            () -> -1*applyDeadband(m_driverController.getLeftY(), 0.2), 
            () -> -1*applyDeadband(m_driverController.getLeftX(), 0.2),
            () -> -1*applyDeadband(m_driverController.getRightX(), 0.2)
    );
    swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
	
	    // a for shoot
    BasicShoot ts = new BasicShoot(SHOOTER);
    m_driverController.a().whileTrue(ts);

    // b for intake
    BasicIntake ti = new BasicIntake(SHOOTER, INTAKE);
    if (!m_driverController.a().getAsBoolean()) {
      m_driverController.b().whileTrue(ti);
    }

  }

  private double applyDeadband(double value, double deadband) {
    if(Math.abs(value) > deadband) {
      return value;
    m_operatorController.x().whileTrue(INTAKE.intakePivotRaw(.05));
    }
    // y for shooter pivot up
    m_operatorController.y().whileTrue(SHOOTER.setShooterPivotraw(.05));
    return 0;
    // left bumper for shooter pivot pid test
    m_operatorController.leftBumper().whileTrue(SHOOTER.setShooterPivotangle(60));

    // right bumper for intake pivot pid test
    m_operatorController.rightBumper().whileTrue(INTAKE.setIntakePivotAngle(90));

    // for testing to make sure we dont need to invert
    //
    // x for intake pivot up
    m_operatorController.x().whileTrue(INTAKE.intakePivotRaw(.05));

    // y for shooter pivot up
    m_operatorController.y().whileTrue(SHOOTER.setShooterPivotraw(.05));

    // left bumper for shooter pivot pid test
    m_operatorController.leftBumper().whileTrue(SHOOTER.setShooterPivotangle(60));

    // right bumper for intake pivot pid test
    m_operatorController.rightBumper().whileTrue(INTAKE.setIntakePivotAngle(90));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // // Choreo swerve auto
    return null;
    // return Autos.choreoSwerveAuto(m_swerveDrive, "NewPath");
  }
}
