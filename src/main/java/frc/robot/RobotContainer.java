// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final Shooter m_shooter = new Shooter();
  private final SwerveSubsystem swerveSubsystem;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/maxSwerve"));
    // Configure the trigger bindings
    // TODO: Register each command that could be run as part of an auto path here (don't delete this todo)
    // NamedCommands.registerCommand("commandName", getAutonomousCommand());
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Calls the command ZeroGyro when the right bumper on the drivers controller is pressed
    //DriveShooter shooterCommand = new DriveShooter(m_shooter, m_driverController::getRightTriggerAxis, m_driverController::getLeftTriggerAxis);
    //m_shooter.setDefaultCommand(shooterCommand);
    // DriveSwerve driveCommand = new DriveSwerve(swerveSubsystem);

    // This command works for sim, there is no need for a separate sim drive command
    // The sim drive command's angle is position-based and not commanded by angular velocity, so this should be used regardless
    Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveCommand(
            () -> -1*applyDeadband(m_driverController.getLeftY(), 0.2), 
            () -> -1*applyDeadband(m_driverController.getLeftX(), 0.2),
            () -> -1*applyDeadband(m_driverController.getRightX(), 0.2)
    );
    swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }

  private double applyDeadband(double value, double deadband) {
    if(Math.abs(value) > deadband) {
      return value;
    }
    return 0;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // // Choreo swerve auto
    return null;
    ChoreoTrajectory traj = Choreo.getTrajectory("NewPath");

    return Choreo.choreoSwerveCommand(
      traj,
      swerveSubsystem.getPose(),
      new PIDController(swerveSubsystem.getSwerveDrive().swerveController.config.headingPIDF.p, 0.0, 0.0), // 
      new PIDController(swerveSubsystem.getSwerveDrive().swerveController.config.headingPIDF.p, 0.0, 0.0), // 
      // TODO: Find out if this is defined anywhere else (robot angle pid)
      // may want to tune all these pids separately from what's in the heading config
      new PIDController(swerveSubsystem.getSwerveDrive().getModules()[0].getConfiguration().anglePIDF.p, 0.0, 0.0), //,
      null,
      null,
      null)
  }
}
