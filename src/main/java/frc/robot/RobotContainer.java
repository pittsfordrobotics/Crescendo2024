// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AutoActionCommands.AutoShoot;
import frc.robot.commands.AutoActionCommands.AutoShootSubwoof;
import frc.robot.commands.AutoActionCommands.StartIntakeCommand;
import frc.robot.commands.AutoActionCommands.StartIntakeNoDelaysCommand;
import frc.robot.commands.AutoActionCommands.AutoFireNote;
import frc.robot.commands.DisabledInstantCommand;
import frc.robot.commands.NewPrettyCommands.*;
import frc.robot.lib.AutoCommandFactory;
import frc.robot.lib.FFCalculator;
import frc.robot.lib.StructureStates;
import frc.robot.lib.StructureStates.structureState;
import frc.robot.lib.util.AllianceFlipUtil;
import frc.robot.lib.util.ShooterInterpolationHelper;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import frc.robot.subsystems.Vision.Vision;

import com.pathplanner.lib.path.PathPlannerPath;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem;
  private final SendableChooser<Command> driveModeChooser;
  private final Climber climber;
  private final Shooter shooter;
  private final Intake intake;
  private final Vision vision;
  private double previousRumblePower = 0;

  private final AutoCommandFactory autoCommandFactory;

    private SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(
      OperatorConstants.kOperatorControllerPort);
  Command speakerTargetSteeringCommand;
    private static Pose2d pathPlannerTargetPose;

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    climber = new Climber();
    shooter = new Shooter();
    intake = new Intake();
    swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/maxSwerve"));
    vision = new Vision(VisionConstants.LIMELIGHT1,  VisionConstants.LIMELIGHT2, swerveSubsystem::addVisionData);
      pathPlannerTargetPose = new Pose2d();

        DoubleSupplier distanceSupplier = (() -> swerveSubsystem.getPose().getTranslation().getDistance(AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.getTranslation())));

        DoubleSupplier angleSupplier = ShooterInterpolationHelper.getShooterAngle(distanceSupplier);
        DoubleSupplier RPMSupplier = ShooterInterpolationHelper.getShooterRPM(distanceSupplier);
        Supplier<Pose2d> pathPlannerTargetPoseSupplier = (() -> pathPlannerTargetPose);

        NamedCommands.registerCommand("StartIntakeNoDelaysCommand", new SequentialCommandGroup(
          new StoredCommand(shooter, intake),
          Commands.waitSeconds(0.5),
          new StartIntakeNoDelaysCommand(shooter, intake)
          )
        );
        NamedCommands.registerCommand("AutoFireNote", new AutoFireNote(shooter)); // waits for spinner rpm (MUST be previously set to spin up), then fires note
        NamedCommands.registerCommand("StoredCommand", new StoredCommand(shooter, intake));
        NamedCommands.registerCommand("AimSpeaker", new RepeatCommand(new CommonSpeakerCommandNoDelays(shooter, intake, angleSupplier, RPMSupplier)));
        NamedCommands.registerCommand("ShootSubwoof", new SequentialCommandGroup(
          new SUBWOOFCommand(shooter, intake),
          new AutoFireNote(shooter),
          new StoredCommand(shooter, intake)
        ));
        NamedCommands.registerCommand("ShootSubwoofSide", new SequentialCommandGroup(
          new SUBWOOFCommandSide(shooter, intake),
          new AutoFireNote(shooter),
          new StoredCommand(shooter, intake)
        ));
        NamedCommands.registerCommand("CorrectHeading", swerveSubsystem.correctHeading(pathPlannerTargetPoseSupplier).withTimeout(1.5));
        NamedCommands.registerCommand("CorrectHeadingShortTimeout", swerveSubsystem.correctHeading(pathPlannerTargetPoseSupplier).withTimeout(0.5));
        NamedCommands.registerCommand("AlignStuffOnStart", new SequentialCommandGroup(setGyroBasedOnPathPlannerTrajectory(), swerveSubsystem.resetOdometry(pathPlannerTargetPoseSupplier)));
        // instantiates autoChooser based on PathPlanner files (exists at code deploy, no need to wait)
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("PathPlanner Auto Chooser", autoChooser);

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> { 
        pathPlannerTargetPose = pose;
        });

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

    Pose2d speaker = FieldConstants.Speaker.centerSpeakerOpening;
    speakerTargetSteeringCommand = swerveSubsystem.driveTranslationAndPointAtTarget(
        () -> -m_driverController.getLeftY(),
        () -> -m_driverController.getLeftX(),
        speaker);
    rotationRateSteeringCommand.setName("Rotation Rate Steer");
    driveModeChooser.setDefaultOption("Enhanced Steering (BETA)", enhancedHeadingSteeringCommand);
    driveModeChooser.addOption("Heading Steering", headingSteeringCommand);
    driveModeChooser.addOption("Rotation Rate Steering", rotationRateSteeringCommand);
    Shuffleboard.getTab("CONFIG").add(driveModeChooser);
    DisabledInstantCommand zeroOffsetCommand = new DisabledInstantCommand(swerveSubsystem::setSwerveOffsets);
    zeroOffsetCommand.setName("Zero Offsets");
    Shuffleboard.getTab("CONFIG").add("Zero Swerve Module Offsets", zeroOffsetCommand);

    StructureStates.setCurrentState(StructureStates.structureState.startup);
    // Configure the trigger bindings
    configure_COMP_Bindings();
    //configure_TEST_Bindings();
    autoConfig();
  }

  private void configure_COMP_Bindings() {
    // ToDo:
    // Test if stored command should be set in the begining or end of the command

    // Swerve
    m_driverController.start().onTrue(new DisabledInstantCommand(swerveSubsystem::zeroGyro));

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

    // upgraded point and aim at speaker
    DoubleSupplier distanceSupplier = (() -> swerveSubsystem.getPose().getTranslation()
        .getDistance(FieldConstants.allianceFlipper(new Pose3d(FieldConstants.Speaker.centerSpeakerOpeningZeroX),
            DriverStation.getAlliance().get()).toPose2d().getTranslation()));

    m_driverController.a().whileTrue(speakerTargetSteeringCommand.alongWith(new RepeatCommand(
        new CommonSpeakerCommand(shooter, intake,
            ShooterInterpolationHelper.getShooterAngle(distanceSupplier),
            ShooterInterpolationHelper.getShooterRPM(distanceSupplier)))));

    m_driverController.a().onFalse(new SequentialCommandGroup(
        shooter.spinIndexerCommand(RobotConstants.INDEXER_SHOOT_SPEED),
        new WaitCommand(.25),
        shooter.spinIndexerCommand(RobotConstants.INDEXER_IDLE_SPEED),
        new StoredCommand(shooter, intake)));

    // old point at speaker
    // m_driverController.a().whileTrue(speakerTargetSteeringCommand);

    // Runs the indexer while the right bumper is held -- essentally a shoot command
    m_driverController.rightBumper().onTrue(shootIndexerCommand);
    m_driverController.rightBumper().onFalse(Commands.runOnce(() -> {
      idleIndexerCommand.schedule();
      storedCommand.schedule();
    }));

    PathPlannerPath redampPath = PathPlannerPath.fromPathFile("RedAmpPath");
    PathPlannerPath blueampPath = PathPlannerPath.fromPathFile("BlueAmpPath");
    Command blueampheadingcommand = swerveSubsystem.correctHeading(Rotation2d.fromDegrees(-90))
        .beforeStarting(Commands.runOnce(() -> vision.useVision(false)));
    Command redampheadingcommand = swerveSubsystem.correctHeading(Rotation2d.fromDegrees(90))
        .beforeStarting(Commands.runOnce(() -> vision.useVision(false)));

    m_driverController.b().onTrue(
        new ParallelCommandGroup(
            new BetterAMPCommand(shooter, intake)
            // new ConditionalCommand(swerveSubsystem.pathToPath(blueampPath), swerveSubsystem.pathToPath(redampPath),
            //     () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue))
            // .beforeStarting(new ConditionalCommand(blueampheadingcommand, redampheadingcommand,
            //     () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue)
            //     .withTimeout(1)
            ));
    m_driverController.b().onFalse(
        new SequentialCommandGroup(
            //Commands.runOnce(() -> vision.useVision(true)),
            AmpShootIntake,
            new WaitCommand(.75),
            new StoredCommand(shooter, intake)));

    m_driverController.b().onTrue(Commands.runOnce(() -> swerveSubsystem.setTargetAngle(
        getAllianceDefaultBlue().equals(Alliance.Red) ? Rotation2d.fromDegrees(90) : Rotation2d.fromDegrees(-90))));

    // Old amp scoring approach
    // Runs the intake on left bummper true
    m_driverController.leftBumper().onTrue(intake.spinIntakeCommand(RobotConstants.NEWAMP_IntakeSpeed_ShootOut));
    m_driverController.leftBumper().onFalse(storedCommand);
    m_operatorController.x().onTrue(betterAmpCommand);

    m_operatorController.b().onTrue(subwoofCommand);
    m_operatorController.y().onTrue(podiumCommand);

    m_driverController.x().onTrue(Commands.runOnce(() -> {
      if (StructureStates.getCurrentState() == structureState.stored) {
        intakeCommand.schedule();
      } else {
        storedCommand.schedule();
      }
    }));

    m_operatorController.rightBumper().onTrue(climber.setSpeedCommand(0.5));
    m_operatorController.rightBumper().onFalse(climber.setSpeedCommand(-0.5));
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
    SmartDashboard.putNumber("IntakePivotAngle_CHANGEME", 35);

    // LEFT BUMPER & TRIGGER -> shooter pivot -- Works (tune pids and FF tho)
    m_operatorController.leftBumper().onTrue(shooter.setPivotAngleCommand(53));
    m_operatorController.leftBumper().onFalse(shooter.setPivotAngleCommand(0.0));
    m_operatorController.leftTrigger().whileTrue(shooter.setPivotAngleSupplierCommand());

    // RIGHT BUMPER & TRIGGER -> intake pivot -- Works (tune pids and FF tho)
    m_operatorController.rightBumper().onFalse(intake.setPivotAngleCommand(180));
    m_operatorController.rightBumper().onTrue(intake.setPivotAngleCommand(0));
    m_operatorController.rightTrigger().whileTrue(intake.setPivotAngleSupplierCommand());

    // A -> Shooter RPM (X for supplier) -- Works
    m_operatorController.a().onTrue(shooter.setShooterRPMCommand(5400));
    m_operatorController.a().onFalse(shooter.setShooterRPMCommand(-2500));
    m_operatorController.x().whileTrue(shooter.setShooterRPMSupplierCommand());

    // B -> Intake RAW command -- Untested
    m_operatorController.b().onTrue(intake.spinIntakeCommand(.7));
    m_operatorController.b().onFalse(intake.spinIntakeCommand(0));

    // Y -> Indexer test -- Works
    m_operatorController.y().onTrue(shooter.spinIndexerCommand(1));
    m_operatorController.y().onFalse(shooter.spinIndexerCommand(-0.1));
  }

  // private ChoreoTrajectory finalAutoTrajectory;
  // public void setGyroBasedOnAutoFinalTrajectory(){
  // swerveSubsystem.setGyroAngle(finalAutoTrajectory.getFinalPose().getRotation());
  // // Sets the gyro heading, NEVER FLIPPED since robot should always point away
  // from DS when gyro reports 0
  // } // Unnecessary since it can be done initially

  /**
   * <p>
   * Sets the gyro heading, flipped in relation to the ALLIANCE's origin, NOT the
   * origin of the FIELD.
   * </p>
   * <p>
   * Uses alliance-relative flipping, so only used for teleop.
   * </p>
   * <p>
   * Auto will simply ignore it if odometry is reset after.
   * </p>
   * 
   * @param traj The choreo trajectory whose initial angle is used to set the gyro
   *             angle.
   */
  public void setGyroBasedOnInitialChoreoTrajectory(ChoreoTrajectory traj) {
    System.out.println("Is alliance present when setting initial gyro? " + DriverStation.getAlliance().isPresent());
    Rotation2d actualAllianceRelativeAngle = DriverStation.getAlliance().get() == Alliance.Blue
        ? traj.getInitialPose().getRotation()
        : new Rotation2d().minus(traj.getInitialPose().getRotation());
    swerveSubsystem.setGyroAngle(actualAllianceRelativeAngle);
  }

  public Command setGyroBasedOnPathPlannerTrajectory(){
    return Commands.runOnce(() -> {
      System.out.println("Is alliance present when setting initial gyro? " + DriverStation.getAlliance().isPresent());
      System.out.println("What is the perceived initial pathplanner pose?:" + pathPlannerTargetPose.getTranslation() + " " + (pathPlannerTargetPose.getRotation()));
      Rotation2d actualFieldRelativeRotation = pathPlannerTargetPose.getRotation();
      Rotation2d allianceRelativeRotation = DriverStation.getAlliance().get() == Alliance.Blue ? actualFieldRelativeRotation : actualFieldRelativeRotation.plus(Rotation2d.fromDegrees(180));
      swerveSubsystem.setGyroAngle(allianceRelativeRotation);
    });
  }

  public static Pose2d getPathPlannerTargetPose() {
    return pathPlannerTargetPose;
  }

  /**
   * Requires the swerve subsystem, w/ tolerance of 1 degree and timeout of 2
   * secs.
   * 
   * @return Command to drive to zero heading with no translation rate and zeros
   *         the gyro.
   */
  public Command driveToZeroHeadingAndZeroGyro() {
    return swerveSubsystem.headingDriveCommand(() -> 0, () -> 0, Rotation2d::new)
        .until(() -> Math.abs(swerveSubsystem.getGyroYaw().getDegrees()) < 1).withTimeout(2)
        .andThen(swerveSubsystem::zeroGyro);
  }

  public Command zeroOdometryAngleOffset() {
    return swerveSubsystem.zeroOdometryAngleOffset();
  }

  public Command zeroOdometryFromLastPathPose() {
    return swerveSubsystem.zeroOdometryFromLastPathPose();
  }

  public Alliance getAllianceDefaultBlue() {
    Alliance currentAlliance;
    if (DriverStation.getAlliance().isPresent()) {
      currentAlliance = DriverStation.getAlliance().get();
    } else {
      currentAlliance = Alliance.Blue;
      System.out.println("No alliance, setting to blue");
    }
    return currentAlliance;
  }

  public void buzz_controllers(double power) {
    if (Math.abs(power - previousRumblePower) < .1) {
      return;
    }
    m_driverController.getHID().setRumble(RumbleType.kBothRumble, power);
    m_operatorController.getHID().setRumble(RumbleType.kBothRumble, power);
  }

  public Command buzz_timed (double power, double time) {
    return Commands.run(() -> buzz_controllers(power)).withTimeout(time).andThen(() -> buzz_controllers(0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   */
  public void autoConfig() {
    
    // /* TWONOTEMIDDLE START */
    // ChoreoTrajectory twonotemiddletraj1 = Choreo.getTrajectory("twonotemiddle.1");
    // ChoreoTrajectory twonotemiddletraj2 = Choreo.getTrajectory("twonotemiddle.2");
    // ChoreoTrajectory twonotemiddletraj3 = Choreo.getTrajectory("twonotemiddle.3");
    // ChoreoTrajectory twonotemiddletraj4 = Choreo.getTrajectory("twonotemiddle.4");

    // Command twonotemiddle = new SequentialCommandGroup(
    //   Commands.runOnce(() -> {
    //       setGyroBasedOnInitialChoreoTrajectory(twonotemiddletraj1);
    //     if(DriverStation.getAlliance().get() == Alliance.Blue) {
    //       swerveSubsystem.resetOdometry(twonotemiddletraj1.getInitialPose());
    //     } else {
    //       swerveSubsystem.resetOdometry(twonotemiddletraj1.flipped().getInitialPose());
    //     }
    //   }),
    //   new AutoShoot(shooter, intake, 56, 5400),
    //   autoCommandFactory.generateChoreoCommand(twonotemiddletraj1),
    //   swerveSubsystem.correctHeading(twonotemiddletraj1).withTimeout(1.5),
    //   new StartIntakeCommand(shooter, intake), // Start the intake
    //   autoCommandFactory.generateChoreoCommand(twonotemiddletraj2), // Drive forward
    //   shooter.waitForLimitSwitchCommand().withTimeout(3), // Wait for it to intake
    //   new StoredCommand(shooter, intake), // Store the note
    //   autoCommandFactory.generateChoreoCommand(twonotemiddletraj3), // Turn towards the speaker
    //   swerveSubsystem.correctHeading(twonotemiddletraj3).withTimeout(1.5),
    //   new AutoShoot(shooter, intake, 45, 6000), // Ready to shoot again (adjust these params)
    //   autoCommandFactory.generateChoreoCommand(twonotemiddletraj4) // drive out of starting area fully
    // );
    // autoChooser.addOption("Two Note Middle", twonotemiddle);
    // /* TWONOTEMIDDLE END */

    // /* TWONOTEBOTTOM START */
    // ChoreoTrajectory twonotebottomtraj1 = Choreo.getTrajectory("twonotebottom.1");
    // ChoreoTrajectory twonotebottomtraj2 = Choreo.getTrajectory("twonotebottom.2");
    // ChoreoTrajectory twonotebottomtraj3 = Choreo.getTrajectory("twonotebottom.3");
    // ChoreoTrajectory twonotebottomtraj4 = Choreo.getTrajectory("twonotebottom.4");

    // Command twonotebottom = new SequentialCommandGroup(
    //   Commands.runOnce(() -> {
    //       setGyroBasedOnInitialChoreoTrajectory(twonotebottomtraj1);
    //     if(DriverStation.getAlliance().get() == Alliance.Blue) {
    //       swerveSubsystem.resetOdometry(twonotebottomtraj1.getInitialPose());
    //     } else {
    //       swerveSubsystem.resetOdometry(twonotebottomtraj1.flipped().getInitialPose());
    //     }
    //   }),
    //   new AutoShootSubwoof(shooter, intake),
    //   autoCommandFactory.generateChoreoCommand(twonotebottomtraj1),
    //   swerveSubsystem.correctHeading(twonotemiddletraj1).withTimeout(1.5),
    //   new StartIntakeCommand(shooter, intake),
    //   autoCommandFactory.generateChoreoCommand(twonotebottomtraj2),
    //   shooter.waitForLimitSwitchCommand().withTimeout(3), // Wait for it to intake
    //   new StoredCommand(shooter, intake),
    //   autoCommandFactory.generateChoreoCommand(twonotebottomtraj3),
    //   swerveSubsystem.correctHeading(twonotebottomtraj3).withTimeout(1.5),
    //   new CommonSpeakerCommand(shooter, intake, 41.5, 6000), // Ready to shoot again (adjust these params)
    // shooter.waitForShooterRPMCommand().withTimeout(1),
    //   shooter.spinIndexerCommand(RobotConstants.INDEXER_SHOOT_SPEED), // Shoot
    //   Commands.waitSeconds(0.25),
    //   shooter.spinIndexerCommand(RobotConstants.INDEXER_IDLE_SPEED),
    //   new StoredCommand(shooter, intake),
    //autoCommandFactory.generateChoreoCommand(twonotebottomtraj4)
    // );
    // autoChooser.addOption("Two Note Podiumside", twonotebottom);
    // /* TWONOTEBOTTOM START */

    // /* TWONOTETOP START */
    // ChoreoTrajectory twonotetoptraj1 = Choreo.getTrajectory("twonotetop.1");
    // ChoreoTrajectory twonotetoptraj2 = Choreo.getTrajectory("twonotetop.2");
    // ChoreoTrajectory twonotetoptraj3 = Choreo.getTrajectory("twonotetop.3");

    // Command twonotetop = new SequentialCommandGroup(
    //   Commands.runOnce(() -> {
    //       setGyroBasedOnInitialChoreoTrajectory(twonotetoptraj1);
    //     if(DriverStation.getAlliance().get() == Alliance.Blue) {
    //       swerveSubsystem.resetOdometry(twonotetoptraj1.getInitialPose());
    //     } else {
    //       swerveSubsystem.resetOdometry(twonotetoptraj1.flipped().getInitialPose());
    //     }
    //   }),
    //   new AutoShootSubwoof(shooter, intake),
    //   autoCommandFactory.generateChoreoCommand(twonotetoptraj1),
    //   swerveSubsystem.correctHeading(twonotemiddletraj1).withTimeout(1.5),
    //   new StartIntakeCommand(shooter, intake),
    //   autoCommandFactory.generateChoreoCommand(twonotetoptraj2),
    //   shooter.waitForLimitSwitchCommand().withTimeout(3), // Wait for it to intake
    //   new StoredCommand(shooter, intake),
    //   autoCommandFactory.generateChoreoCommand(twonotetoptraj3),
    //   swerveSubsystem.correctHeading(twonotetoptraj3).withTimeout(1.5),
    //   new CommonSpeakerCommand(shooter, intake, 38.7, 6000), // Ready to shoot again (adjust these params)
    // shooter.waitForShooterRPMCommand().withTimeout(1),
    //   shooter.spinIndexerCommand(RobotConstants.INDEXER_SHOOT_SPEED), // Shoot
    //   Commands.waitSeconds(0.25),
    //   shooter.spinIndexerCommand(RobotConstants.INDEXER_IDLE_SPEED),
    //   new StoredCommand(shooter, intake)
    // );
    // autoChooser.addOption("Two Note Ampside", twonotetop);
    // /* TWONOTETOP STOP */

      /* START ONENOTEBOTTOMBERSERK */
    //   ChoreoTrajectory onenotebottomberserktraj1 = Choreo.getTrajectory("onenotebottomandmiddlerow.1");
    //   Command oneNoteBottomBerserk = new SequentialCommandGroup(
    //           Commands.runOnce(() -> {
    //               setGyroBasedOnInitialChoreoTrajectory(onenotebottomberserktraj1);
    //               if(DriverStation.getAlliance().get() == Alliance.Blue) {
    //                   swerveSubsystem.resetOdometry(onenotebottomberserktraj1.getInitialPose());
    //               } else {
    //                   swerveSubsystem.resetOdometry(onenotebottomberserktraj1.flipped().getInitialPose());
    //               }
    //           }),
    //           new AutoShootSubwoof(shooter, intake),
    //           autoCommandFactory.generateChoreoCommand(onenotebottomberserktraj1)
    //   );
    //   autoChooser.addOption("One Note Podium Side Berserk", oneNoteBottomBerserk);
      /* END ONENOTEBOTTOMBERSERK */

      /* START ONENOTEBOTTOM */
    //   ChoreoTrajectory onenotebottomtraj1 = Choreo.getTrajectory("onenotebottom.1");
    //   Command oneNoteBottom = new SequentialCommandGroup(
    //           Commands.runOnce(() -> {
    //               setGyroBasedOnInitialChoreoTrajectory(onenotebottomtraj1);
    //               if(DriverStation.getAlliance().get() == Alliance.Blue) {
    //                   swerveSubsystem.resetOdometry(onenotebottomtraj1.getInitialPose());
    //               } else {
    //                   swerveSubsystem.resetOdometry(onenotebottomtraj1.flipped().getInitialPose());
    //               }
    //           }),
    //           new AutoShootSubwoof(shooter, intake),
    //           autoCommandFactory.generateChoreoCommand(onenotebottomberserktraj1)
    //   );
    //   autoChooser.addOption("One Note Podium Side Berserk", oneNoteBottom);
      /* END ONENOTEBOTTOM */

      /* START ONENOTETOP */
    //   ChoreoTrajectory onenotetoptraj1 = Choreo.getTrajectory("onenotetop.1");
    //   Command oneNoteTop = new SequentialCommandGroup(
    //           Commands.runOnce(() -> {
    //               setGyroBasedOnInitialChoreoTrajectory(onenotetoptraj1);
    //               if(DriverStation.getAlliance().get() == Alliance.Blue) {
    //                   swerveSubsystem.resetOdometry(onenotetoptraj1.getInitialPose());
    //               } else {
    //                   swerveSubsystem.resetOdometry(onenotetoptraj1.flipped().getInitialPose());
    //               }
    //           }),
    //           new AutoShootSubwoof(shooter, intake),
    //           autoCommandFactory.generateChoreoCommand(onenotetoptraj1)
    //   );
    //   autoChooser.addOption("One Note Ampside", oneNoteTop);
      /* END ONENOTETOP */

    // autoChooser.setDefaultOption("Do nothing", new InstantCommand());
    // SmartDashboard.putData(autoChooser);
  }

  public Command getAutonomousCommand() {
    // return new RunCommand(() -> swerveSubsystem.drive(new Translation2d(.1, .2),
    // .3, false), swerveSubsystem); // test drive command, for debugging
    //return autoChooser.getSelected().andThen(zeroOdometryAngleOffset());
    return autoChooser.getSelected();
    // TODO: Replace the andThen statement if this does not align to gyro
  }
}
