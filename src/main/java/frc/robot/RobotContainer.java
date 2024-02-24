// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoActionCommands.StartIntakeCommand;
import frc.robot.commands.DisabledInstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.NewPrettyCommands.*;
import frc.robot.lib.AutoCommandFactory;
import frc.robot.commands.NewPrettyCommands.*;
import frc.robot.commands.NewPrettyCommands.IntakeCommand;
import frc.robot.commands.NewPrettyCommands.PODIUMCommand;
import frc.robot.commands.NewPrettyCommands.SUBWOOFCommand;
import frc.robot.commands.NewPrettyCommands.StoredCommand;
import frc.robot.lib.FFCalculator;
import frc.robot.lib.StructureStates;
import frc.robot.lib.StructureStates.structureState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.io.File;
import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem swerveSubsystem;
    private final SendableChooser<Command> driveModeChooser;
    private final Climber climber;
    private final Shooter shooter;
    private final Intake intake;

  private final AutoCommandFactory autoCommandFactory;

  // TODO: Move subsystem commands here

 private final SendableChooser<Command> autoChooser = new SendableChooser<>();
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
    c.updateIntakePivotAngle(intake::getPivotAngleDeg);
    c.updateShooterAngle(shooter::getPivotAngleDeg);
    driveModeChooser = new SendableChooser<>();

    autoCommandFactory = new AutoCommandFactory(swerveSubsystem);
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
    driveModeChooser.setDefaultOption("Enhanced Steering (BETA)", enhancedHeadingSteeringCommand);
    driveModeChooser.addOption("Heading Steering", headingSteeringCommand);
    driveModeChooser.addOption("Rotation Rate Steering", rotationRateSteeringCommand);
    Shuffleboard.getTab("CONFIG").add(driveModeChooser);
    DisabledInstantCommand zeroOffsetCommand = new DisabledInstantCommand(swerveSubsystem::setSwerveOffsets);
    zeroOffsetCommand.setName("Zero Offsets");
    Shuffleboard.getTab("CONFIG").add("Zero Swerve Module Offsets", zeroOffsetCommand);
    // Configure the trigger bindings
    configure_COMP_Bindings();
    // configure_TEST_Bindings();
    autoConfig();

    StructureStates.setCurrentState(StructureStates.structureState.startup);
  }
    private void configure_COMP_Bindings() {
        // Swerve
        m_driverController.start().onTrue(new DisabledInstantCommand(swerveSubsystem::zeroGyro));
        // m_driverController.leftBumper().onTrue(swerveSubsystem.setSlowSpeed()).onFalse(swerveSubsystem.setNormalSpeed());
        Command driveCommand = driveModeChooser.getSelected();
        swerveSubsystem.setDefaultCommand(driveCommand);
        driveModeChooser.onChange(command -> {
            Command currentDefault = swerveSubsystem.getDefaultCommand();
            swerveSubsystem.removeDefaultCommand();
            CommandScheduler.getInstance().cancel(currentDefault);
            swerveSubsystem.setDefaultCommand(command);
            System.out.println(swerveSubsystem.getDefaultCommand().getName());
        });
        m_driverController.start().onTrue(new InstantCommand(() -> {
            swerveSubsystem.zeroGyro();
            System.out.println("Resetting gyro");
        }));

        // // states
        StoredCommand storedCommand = new StoredCommand(shooter, intake);
        IntakeCommand intakeCommand = new IntakeCommand(shooter, intake);
        BetterAMPCommand betterAmpCommand = new BetterAMPCommand(shooter, intake);
        SUBWOOFCommand subwoofCommand = new SUBWOOFCommand(shooter, intake);
        PODIUMCommand podiumCommand = new PODIUMCommand(shooter, intake);
        Command idleIndexerCommand = shooter.spinIndexerCommand(RobotConstants.INDEXER_IDLE_SPEED);
        Command shootIndexerCommand = shooter.spinIndexerCommand(RobotConstants.INDEXER_SHOOT_SPEED);
        Command AmpShootIntake = intake.spinIntakeCommand(RobotConstants.NEWAMP_IntakeSpeed_ShootOut);

        // Runs the indexer while the right bumper is held -- essentally a shoot command
        m_driverController.rightBumper().onTrue(shootIndexerCommand)
                .onFalse(idleIndexerCommand);
        // Runs the intake on left bummper true
        m_driverController.leftBumper().onTrue(AmpShootIntake);

        m_operatorController.b().onTrue(subwoofCommand);
        m_operatorController.y().onTrue(podiumCommand);
        m_operatorController.x().onTrue(betterAmpCommand);

        m_driverController.x().onTrue(Commands.runOnce(() -> {
            if (StructureStates.getCurrentState() != structureState.intake) {
                intakeCommand.schedule();
            } else {
                storedCommand.schedule();
            }
        }));
        m_driverController.y().onTrue(storedCommand);
        m_operatorController.rightBumper().onTrue(climber.extendCommand());
        m_operatorController.rightBumper().onFalse(climber.retractCommand());
    }

    private void configure_TEST_Bindings() {
        // Swerve Drive Command chooser
        m_driverController.start().onTrue(new DisabledInstantCommand(swerveSubsystem::zeroGyro));
        m_driverController.leftBumper().onTrue(swerveSubsystem.setSlowSpeed())
                .onFalse(swerveSubsystem.setNormalSpeed());

        Command driveCommand = driveModeChooser.getSelected();
        swerveSubsystem.setDefaultCommand(driveCommand);
        driveModeChooser.onChange(command -> {
            Command currentDefault = swerveSubsystem.getDefaultCommand();
            swerveSubsystem.removeDefaultCommand();
            CommandScheduler.getInstance().cancel(currentDefault);
            swerveSubsystem.setDefaultCommand(command);
            System.out.println(swerveSubsystem.getDefaultCommand().getName());
        });

        // y- indexer
        // b- ontrue intake (onfalse zero)
        // left bumper - shooter angle 53 on true 0 on false
        // right bummper - intake angle 35 on trun 0 on false
        // a - ontrue shooter RPM 5400 (0 on flase)

        // Suppliers //
        // left trigger - shooter angle supplier
        // right trigger - intake angle supplier
        // x - shooter RPM supplier

        // Remember zeroed at intake pose
        // +RPM means note goes out & +Angle means move up relative to intake pose
        SmartDashboard.putNumber("ShooterPivotAngle_CHANGEME", 0);
        SmartDashboard.putNumber("ShooterRPM_CHANGEME", 0);
        SmartDashboard.putNumber("IntakePivotAngle_CHANGEME", 180);

        // LEFT BUMPER & TRIGGER -> shooter pivot -- Works (tune pids and FF tho)
        m_operatorController.leftBumper().onTrue(shooter.setPivotAngleCommand(53));
        m_operatorController.leftBumper().onFalse(shooter.setPivotAngleCommand(0.0));
        m_operatorController.leftTrigger().whileTrue(shooter.setPivotAngleSupplierCommand());

        // RIGHT BUMPER & TRIGGER -> intake pivot -- Works (tune pids and FF tho)
        m_operatorController.rightBumper().onFalse(intake.setPivotAngleCommand(35));
        m_operatorController.rightBumper().onTrue(intake.setPivotAngleCommand(0));
        m_operatorController.rightTrigger().whileTrue(intake.setPivotAngleSupplierCommand());

        // A -> Shooter RPM (X for supplier) -- Works
        m_operatorController.a().onTrue(shooter.setShooterRPMCommand(5400));
        m_operatorController.a().onFalse(shooter.setShooterRPMCommand(-2500));
        m_operatorController.x().whileTrue(shooter.setShooterRPMSupplierCommand());
        // B -> Intake RAW command -- Untested
        m_operatorController.b().onTrue(intake.spinIntakeCommand(.7));
        m_operatorController.b().onFalse(intake.spinIntakeCommand(.01));

        // Y -> Indexer test -- Works
        m_operatorController.y().onTrue(shooter.spinIndexerCommand(0.5));
        m_operatorController.y().onFalse(shooter.spinIndexerCommand(-0.1));
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
    Pose2d checkpoint1 = twonotemiddletraj1.getFinalPose();
    Pose2d checkpoint2 = twonotemiddletraj2.getFinalPose();
    Pose2d checkpoint3 = twonotemiddletraj3.getFinalPose();

    Command twonotemiddle = new SequentialCommandGroup(
      Commands.runOnce(() -> {
        if(DriverStation.getAlliance().get() == Alliance.Blue) {
          swerveSubsystem.resetOdometry(twonotemiddletraj1.getInitialPose());
        } else {
          swerveSubsystem.resetOdometry(twonotemiddletraj1.flipped().getInitialPose());
        }
      }),
      new SUBWOOFCommand(shooter, intake),
      Commands.waitSeconds(0.5), // move to position and spin up
      shooter.spinIndexerCommand(RobotConstants.INDEXER_SHOOT_SPEED),
      Commands.waitSeconds(0.25),
      shooter.spinIndexerCommand(RobotConstants.INDEXER_IDLE_SPEED), // Idle indexer and prepare to intake
      autoCommandFactory.generateChoreoCommand(twonotemiddletraj1),
      Commands.waitSeconds(2),
      swerveSubsystem.correctHeading(checkpoint1.getRotation()).withTimeout(2),
      new StartIntakeCommand(shooter, intake), // Start the intake
      autoCommandFactory.generateChoreoCommand(twonotemiddletraj2), // Drive forward
      shooter.waitForLimitSwitchCommand().withTimeout(5), // Wait for it to intake
      new StoredCommand(shooter, intake), // Store the note
      autoCommandFactory.generateChoreoCommand(twonotemiddletraj3), // Turn towards the speaker
      new CommonSpeakerCommand(shooter, intake, 40, 35, 5900), // Ready to shoot again (adjust these params)
      shooter.waitForPivotAngleCommand(),
      shooter.waitForShooterRPMCommand(),
      shooter.spinIndexerCommand(RobotConstants.INDEXER_SHOOT_SPEED), // Shoot
      Commands.waitSeconds(0.25),
      shooter.spinIndexerCommand(RobotConstants.INDEXER_IDLE_SPEED),
      autoCommandFactory.generateChoreoCommand(twonotemiddletraj4), // drive out of starting area fully
      new StoredCommand(shooter, intake)
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
