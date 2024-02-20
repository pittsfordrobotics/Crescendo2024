// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DisabledInstantCommand;
import frc.robot.commands.AutoActionCommands.AutoSpeakerCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.FieldConstants.Speaker;
import frc.robot.commands.NewPrettyCommands.AmpCommand;
import frc.robot.commands.NewPrettyCommands.IntakeCommand;
import frc.robot.commands.NewPrettyCommands.SpeakerCommand;
import frc.robot.commands.NewPrettyCommands.StoredCommand;
import frc.robot.lib.AutoCommandFactory;
import frc.robot.lib.FFCalculator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.Climber;
import java.io.File;
import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem;
  private final SendableChooser<Command> toggleDriveMode;
  private final Climber CLIMBER = new Climber();
  private final Shooter SHOOTER = new Shooter();
  private final Intake INTAKE = new Intake();

  private final AutoCommandFactory autoCommandFactory;

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

    autoCommandFactory = new AutoCommandFactory(swerveSubsystem);

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
    autoConfig();
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
    ChoreoTrajectory twonotemiddletraj1 = Choreo.getTrajectory("twonotemiddle.1");
    ChoreoTrajectory twonotemiddletraj2 = Choreo.getTrajectory("twonotemiddle.2");
    ChoreoTrajectory twonotemiddletraj3 = Choreo.getTrajectory("twonotemiddle.3");
    ChoreoTrajectory twonotemiddletraj4 = Choreo.getTrajectory("twonotemiddle.4");
    AutoSpeakerCommand shootSubwooferCommand = new AutoSpeakerCommand(SHOOTER, INTAKE);
    AutoSpeakerCommand shootPodiumCommand = new AutoSpeakerCommand(SHOOTER, INTAKE);
    // Intake command here after fixed
    Command twonotemiddle = new SequentialCommandGroup(
      Commands.runOnce(() -> {
        if(DriverStation.getAlliance().get() == Alliance.Blue) {
          swerveSubsystem.resetOdometry(twonotemiddletraj1.getInitialPose());
        } else {
          swerveSubsystem.resetOdometry(twonotemiddletraj1.flipped().getInitialPose());
        }
      }),
      // shoot (robot is right against speaker)
      shootSubwooferCommand,
      autoCommandFactory.generateChoreoCommand(twonotemiddletraj1),
      new ParallelCommandGroup(
        autoCommandFactory.generateChoreoCommand(twonotemiddletraj2)
        // intake, end when note collected
        ),
      autoCommandFactory.generateChoreoCommand(twonotemiddletraj3),
      // shoot (robot is not against speaker)
      shootPodiumCommand,
      autoCommandFactory.generateChoreoCommand(twonotemiddletraj4) // drive out of starting area fully
    );

    autoChooser.setDefaultOption("Two Note Middle", twonotemiddle);
    autoChooser.addOption("Do nothing", new InstantCommand());
    SmartDashboard.putData(autoChooser);
  }
  public Command getAutonomousCommand() {
    // return new RunCommand(() -> swerveSubsystem.drive(new Translation2d(.1, .2), .3, false), swerveSubsystem); // test drive command, for debugging
    return autoChooser.getSelected();
  }
}
