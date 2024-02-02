// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.io.File;
import java.util.Optional;
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
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Routine");

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
    autoConfig();
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
  public void autoConfig() {
    // // Choreo swerve auto
    ChoreoTrajectory traj = Choreo.getTrajectory("NewPath");


    // TODO: Replace these with X and Y translate pids (should be same) and rotational pid tuned using shuffleboard when chassis is ready
    double xTranslateP = 0;
    double yTranslateP = 0;
    double angleP = 0;

    // Angle PID controller
    var thetaController = new PIDController(angleP, 0.0, 0.0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //swerveSubsystem.getSwerveDrive().swerveController.config.headingPIDF.p may be useful
    Command swerveCommand = Choreo.choreoSwerveCommand(
      traj,
      swerveSubsystem::getPose,
      new PIDController(xTranslateP, 0.0, 0.0), // PIDController for field-relative X translation (input: Y error in meters, output: m/s).
      new PIDController(yTranslateP, 0.0, 0.0), // PIDController for field-relative Y translation (input: Y error in meters, output: m/s).
      thetaController, //, PID controller to correct for rotation error
      (ChassisSpeeds speeds) -> swerveSubsystem.drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), speeds.omegaRadiansPerSecond, false), // A function that consumes the target robot-relative chassis speeds and commands them to the robot.
      () -> {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
          return alliance.isPresent() && (alliance.get() == Alliance.Red);
      }, // Whether or not to mirror the path based on alliance (assumes path created for blue)
      swerveSubsystem // Subsystems to require, typically drivetrain only
      );

      swerveCommand = Commands.sequence(
        Commands.runOnce(() -> swerveSubsystem.resetOdometry(traj.getInitialPose())),
        swerveCommand);

      autoChooser.addDefaultOption("test default auto", swerveCommand);

      //SmartDashboard.putData(autoChooser);
  }
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
