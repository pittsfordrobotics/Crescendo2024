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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DisabledInstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.Climber;
import java.io.File;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.simulation.*;
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem;
  private final SendableChooser<Command> toggleDriveMode;
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
    AmpCommand ampCommand = new AmpCommand(SHOOTER, INTAKE);
    m_operatorController.a().onTrue(ampCommand);

    SpeakerCommand speakerCommand = new SpeakerCommand(SHOOTER, INTAKE);
    m_operatorController.b().onTrue(speakerCommand);

    IntakeCommand intakeCommand = new IntakeCommand(SHOOTER, INTAKE);
    m_operatorController.x().onTrue(intakeCommand);

    StoredCommand storedCommand = new StoredCommand(SHOOTER, INTAKE);
    m_operatorController.y().onTrue(storedCommand);

    // Runs the indexer while the right bumper is held -- essentally a shootcommand
    m_driverController.rightBumper().whileTrue(SHOOTER.setIndexer(RobotConstants.INDEXER_SHOOT_SPEED)).whileFalse(SHOOTER.setIndexer(RobotConstants.INDEXER_IDLE_SPEED));

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

    // Remember zeroed at intake pose -- +RPM means note goes out -- +Angle means
    // move up relative to intake pose
    SmartDashboard.putNumber("ShooterPivotAngle_CHANGEME", 0);
    SmartDashboard.putNumber("ShooterRPM_CHANGEME", 0);
    SmartDashboard.putNumber("IntakePivotAngle_CHANGEME", 180);

    // left bumper for shooter pivot pid test -- Works (tune pids and FF tho)
    m_operatorController.leftBumper().onTrue(SHOOTER.setShooterPivotangle(53));
    m_operatorController.leftBumper().onFalse(SHOOTER.setShooterPivotangle(0.0));
    // left trigger for shooter pivot based on suplier -- untested
    m_operatorController.leftTrigger().whileTrue(
        SHOOTER.setShooterPivotangleSupplier(() -> SmartDashboard.getNumber("ShooterPivotAngle_CHANGEME", 0)));

    // right bumper for intake pivot pid test -- Works (tune pids and FF tho)
    m_operatorController.rightBumper().onFalse(INTAKE.setIntakePivotAngle(180));
    m_operatorController.rightBumper().onTrue(INTAKE.setIntakePivotAngle(0));
    // right trigger for intake pivot based on suplier -- untested
    m_operatorController.rightTrigger().whileTrue(
        INTAKE.setIntakePivotAngleSupplier(() -> SmartDashboard.getNumber("IntakePivotAngle_CHANGEME", 170)));

    // Shooter PID RPM -- Works
    m_operatorController.a().onTrue(SHOOTER.setshooterRPM(5400));
    m_operatorController.a().onFalse(SHOOTER.setshooterRPM(-1000));
    // Shooter RPM based on supplier -- untested
    m_operatorController.x()
        .onTrue(SHOOTER.setshooterRPMSupplier(() -> SmartDashboard.getNumber("ShooterRPM_CHANGEME", 0)));
    m_operatorController.x().onFalse(SHOOTER.setshooterRPM(-1000));

    // Intake RAW command -- Untested
    m_operatorController.b().onTrue(INTAKE.setIntakeRpmRAW(0.7));
    m_operatorController.b().onFalse(INTAKE.setIntakeRpmRAW(0));

    // Indexer test -- Works
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
    // TODO: Replace these with X and Y translate pids (should be same) and rotational pid tuned using shuffleboard when chassis is ready
    // swerveSubsystem.getSwerveDrive().swerveController.config.headingPIDF.p may be useful
    double xTranslateP = 0;
    double yTranslateP = 0;
    double angleP = 0;

    // PID controllers
    var xController = new PIDController(xTranslateP, 0.0, 0.0); // PIDController for field-relative X translation (input: Y error in meters, output: m/s).
    var yController = new PIDController(yTranslateP, 0, 0); // PIDController for field-relative Y translation (input: Y error in meters, output: m/s).
    var thetaController = new PIDController(angleP, 0.0, 0.0); // PID controller to correct for rotation error
    thetaController.enableContinuousInput(-Math.PI, Math.PI); // A function that consumes the target robot-relative chassis speeds and commands them to the robot.
    Consumer<ChassisSpeeds> speedsConsumer = (ChassisSpeeds speeds) -> swerveSubsystem.drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), speeds.omegaRadiansPerSecond, false); // Consumes target robot-relative chassis speeds and commands them to the robot
    // Supplier<Pose2d> poseSupplier = swerveSubsystem::getPose;// Robot pose2d supplier
    BooleanSupplier allianceSupplier = () -> { // TODO: Use the fact that the alliance is present when the robot becomes disabled but connected
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
          return alliance.isPresent() && (alliance.get() == Alliance.Red);
      };
    ChoreoTrajectory onepiecept1traj = Choreo.getTrajectory("NewPath.1"); // this is ok to be declared since will be calling mirrorpath on it if we duplicate paths for each alliance to avoid crashes in sim
    ChoreoTrajectory onepiecept2traj = Choreo.getTrajectory("NewPath.2");
    Command onepiecept1 = Choreo.choreoSwerveCommand(
      allianceSupplier.getAsBoolean() ? onepiecept1traj : onepiecept1traj.flipped(),
      swerveSubsystem::getPose,
      xController, 
      yController, 
      thetaController,
      speedsConsumer, 
      allianceSupplier, // Whether or not to mirror the path based on alliance (assumes path created for blue)
      swerveSubsystem // Subsystems to require, typically drivetrain only
      );
    Command onepiecept2 = Choreo.choreoSwerveCommand(
      allianceSupplier.getAsBoolean() ? onepiecept2traj : onepiecept2traj.flipped(),
      swerveSubsystem::getPose, 
      xController, 
      yController, 
      thetaController,
      speedsConsumer, 
      allianceSupplier, // Whether or not to mirror the path based on alliance (assumes path created for blue)
      swerveSubsystem // Subsystems to require, typically drivetrain only
      );
    
    //swerveCommand.setName("NewPath.1");
    Command onepiece = new SequentialCommandGroup(
    //shoot
    onepiecept1, 
    Commands.print("run the thing\n\n\n\n\n\n\n\n\n\n"),
    onepiecept2);
    autoChooser.setDefaultOption("NewPath.1", onepiece);
    autoChooser.addOption("Do nothing", new InstantCommand());
    SmartDashboard.putData(autoChooser);

  }
  public Command getAutonomousCommand() {
    // Initial choreo trajectories must be named the same things as their commands
    // System.out.println(autoChooser.getSelected().getName());
    // ChoreoTrajectory initTraj = Choreo.getTrajectory(autoChooser.getSelected().getName());
    // if(DriverStation.getAlliance().get() == Alliance.Blue) {
    //   swerveSubsystem.resetOdometry(initTraj.getInitialPose());
    // } else {
    //   swerveSubsystem.resetOdometry(initTraj.flipped().getInitialPose());
    // }
    return autoChooser.getSelected();
  }
}
