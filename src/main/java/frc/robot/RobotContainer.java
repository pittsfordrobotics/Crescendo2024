// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DisabledInstantCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.simulation.*;
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem;
  private final Climber CLIMBER = new Climber();
  private final Shooter SHOOTER = new Shooter();
  private final Intake INTAKE = new Intake();

 private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(
      OperatorConstants.kOperatorControllerPort);

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    FFCalculator c = FFCalculator.getInstance();
    // c.updateIntakePivotAngle(INTAKE::getIntakePivotAngle_deg);
    c.updateShooterAngle(SHOOTER::getShooterAngle_deg);
    swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/maxSwerve"));
    DisabledInstantCommand zeroOffsetCommand = new DisabledInstantCommand(swerveSubsystem::setSwerveOffsets);
    zeroOffsetCommand.setName("Zero Offsets");
    Shuffleboard.getTab("CONFIG").add("Zero Swerve Module Offsets", zeroOffsetCommand);
    // Configure the trigger bindings
    configure_COMP_Bindings();
    // configure_TEST_Bindings();
    autoConfig();
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
    AmpCommand ampCommand = new AmpCommand(SHOOTER, INTAKE);
    m_driverController.a().onTrue(ampCommand);

    SpeakerCommand speakerCommand = new SpeakerCommand(SHOOTER, INTAKE);
    m_driverController.b().onTrue(speakerCommand);

    IntakeCommand intakeCommand = new IntakeCommand(SHOOTER, INTAKE);
    m_driverController.x().onTrue(intakeCommand);

    StoredCommand storedCommand = new StoredCommand(SHOOTER, INTAKE);
    m_driverController.y().onFalse(storedCommand);

    // Runs the indexer while the right bumper is held -- essentally a shootcommand
    m_driverController.rightBumper().whileTrue(SHOOTER.setIndexer(RobotConstants.INDEXER_SHOOT_SPEED));

    // Climber toggle on x
    m_operatorController.x().toggleOnTrue(CLIMBER.extend());
    m_operatorController.x().toggleOnFalse(CLIMBER.retract());
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
    m_operatorController.a().onTrue(SHOOTER.setshooterRPM(5700*2.5));
    m_operatorController.a().onFalse(SHOOTER.setshooterRPM(-500*2.5));

    // b button for intake test -- Untested
    m_operatorController.b().onTrue(INTAKE.setIntakeRpmRAW(0.7));
    m_operatorController.b().onFalse(INTAKE.setIntakeRpmRAW(0));

    // y button for indexer test -- Tested and works
    m_operatorController.y().onTrue(SHOOTER.setIndexer(0.5));
    m_operatorController.y().onFalse(SHOOTER.setIndexer(-0.1));
  }

  private double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
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
    //swerveSubsystem.resetOdometry(traj.getInitialPose());

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
    swerveCommand.setName("NewPath");
    autoChooser.setDefaultOption("NewPath", swerveCommand);
    autoChooser.addOption("Do nothing", new InstantCommand());
    SmartDashboard.putData(autoChooser);

  }
  public Command getAutonomousCommand() {
    // Initial choreo trajectories must be named the same things as their commands
    System.out.println(autoChooser.getSelected().getName());
    ChoreoTrajectory initTraj = Choreo.getTrajectory(autoChooser.getSelected().getName());
    if(DriverStation.getAlliance().get() == Alliance.Blue) {
      swerveSubsystem.resetOdometry(initTraj.getInitialPose());
    } else {
      swerveSubsystem.resetOdometry(initTraj.flipped().getInitialPose());
    }
    return autoChooser.getSelected();
  }
}
