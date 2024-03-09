// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This code shamelessly stolen from the YAGSL test code at https://github.com/BroncBotz3481/YAGSL-Example/tree/main

package frc.robot.subsystems;

import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.AbsoluteEncoder;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.lib.VisionData;
import frc.robot.lib.util.AllianceFlipUtil;
import frc.robot.lib.AllDeadbands;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

    /**
     * Swerve drive object.
     */
    private final SwerveDrive swerveDrive;
    /**
     * Maximum speed of the robot in meters per second, used to limit acceleration.
     */
    public double maximumSpeed = Units.feetToMeters(14.5);

    private boolean hadbadreading = false;

    private SimpleWidget velocityP;
    private SimpleWidget angleP;
    private SimpleWidget angleD;
    private SimpleWidget headingP;
    private SimpleWidget headingD;

    private double prevVelocityP;
    private double prevAngleP;
    private double prevAngleD;
    private double prevHeadingP;
    private double prevHeadingD;

    private Rotation2d currentTargetAngle = new Rotation2d();
    private double speedFactor = 1;

    /**
     * Initialize {@link SwerveDrive} with the directory provided.
     *
     * @param directory Directory of swerve drive config files.
     */
    public SwerveSubsystem(File directory) {
        // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
        // In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
        // The encoder resolution per motor revolution is 1 per motor revolution.
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8);
        // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO *
        // ENCODER RESOLUTION).
        // In this case the wheel diameter is 4 inches, which must be converted to
        // meters to get meters/second.
        // The gear ratio is 6.75 motor revolutions per wheel rotation.
        // The encoder resolution per motor revolution is 1 per motor revolution.
        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75);
        System.out.println("\"conversionFactor\": {");
        System.out.println("\t\"angle\": " + angleConversionFactor + ",");
        System.out.println("\t\"drive\": " + driveConversionFactor);
        System.out.println("}");

        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
        // objects being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
            // Alternative method if you don't want to supply the conversion factor via JSON
            // files.
            // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
            // angleConversionFactor, driveConversionFactor);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot
                                                 // via angle.

        setupPathPlanner();

        prevVelocityP = getSwerveDriveConfiguration().modules[0].configuration.velocityPIDF.p;
        velocityP = Shuffleboard.getTab("PID Config").add("Velocity P", prevVelocityP);

        prevAngleP = getSwerveDriveConfiguration().modules[0].configuration.anglePIDF.p;
        angleP = Shuffleboard.getTab("PID Config").add("Angle P", prevAngleP);

        prevAngleD = getSwerveDriveConfiguration().modules[0].configuration.anglePIDF.d;
        angleD = Shuffleboard.getTab("PID Config").add("Angle D", prevAngleD);

        prevHeadingP = getSwerveController().thetaController.getP();
        headingP = Shuffleboard.getTab("PID Config").add("Heading P", prevHeadingP);

        prevHeadingD = getSwerveController().thetaController.getD();
        headingD = Shuffleboard.getTab("PID Config").add("Heading D", prevHeadingD);

        Shuffleboard.getTab("PID Config").addString("PIDs", this::getPIDVals);

        Shuffleboard.getTab("CONFIG").add(this); // for debug

        Shuffleboard.getTab("COMP").addBoolean("Vision Ok", this::isvisionOk);
    }

    /**
     * Construct the swerve drive.
     *
     * @param driveCfg      SwerveDriveConfiguration for the swerve.
     * @param controllerCfg Swerve Controller.
     */
    public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
        swerveDrive = new SwerveDrive(driveCfg, controllerCfg, maximumSpeed);
    }

    /**
     * Setup AutoBuilder for PathPlanner.
     */

    public void setupPathPlanner() {
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(5.0, 0.0, 0.0),
                        // Translation PID constants
                        new PIDConstants(swerveDrive.swerveController.config.headingPIDF.p,
                                swerveDrive.swerveController.config.headingPIDF.i,
                                swerveDrive.swerveController.config.headingPIDF.d),
                        // Rotation PID constants
                        4.5,
                        // Max module speed, in m/s
                        swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                        // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig()
                // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    /**
     * Get the path follower with events.
     *
     * @param pathName       PathPlanner path name.
     * @param setOdomToStart Set the odometry position to the start of the path.
     * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
     */
    /*
     * public Command getAutonomousCommand(String pathName, boolean setOdomToStart)
     * {
     * // Load the path you want to follow using its name in the GUI
     * PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
     * 
     * if (setOdomToStart) {
     * resetOdometry(new Pose2d(path.getPoint(0).position, getHeading()));
     * }
     * 
     * // Create a path following command using AutoBuilder. This will also trigger
     * event markers.
     * return AutoBuilder.followPath(path);
     * }
     */

    /**
     * Use PathPlanner Path finding to go to a point on the field.
     *
     * @param pose Target {@link Pose2d} to go to.
     * @return PathFinding command
     */
    public Command driveToPose(Pose2d pose) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                swerveDrive.getMaximumVelocity(), 4.0,
                swerveDrive.getMaximumAngularVelocity(), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
                pose,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel
                    // before attempting to rotate.
        );
    }

    public Command pathToPath (PathPlannerPath path) {
        // Load the path we want to pathfind to and follow
        // PathPlannerPath path = PathPlannerPath.fromPathFile("Example Human Player Pickup");

        // Create the constraints to use while pathfinding. The constraints defined in
        // the path will only be used for the path.
        PathConstraints constraints = new PathConstraints(
                swerveDrive.getMaximumVelocity() * .25, 2.0,
                swerveDrive.getMaximumAngularVelocity(), Units.degreesToRadians(360));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindThenFollowPath(
                path,
                constraints,
                .5 // Rotation delay distance in meters. This is how far the robot should travel
                    // before attempting to rotate.
        );
    }

    /**
     * Corrects heading angle continuously until it is reached and angular velocity
     * is low (Autonomous only)
     * 
     * @return A command to drive to that heading angle between path segments
     */
   public Command correctHeading(ChoreoTrajectory traj) {
        return this.run(() -> 
        {
        swerveDrive.drive(swerveDrive.swerveController.getTargetSpeeds(0, 0, currentTargetAngle.getRadians(),
                getHeading().getRadians(), swerveDrive.getMaximumVelocity()));
        })
                .beforeStarting(() -> {
                    swerveDrive.setHeadingCorrection(true);
                    // currentTargetAngle = (DriverStation.getAlliance().get() == Alliance.Blue) ?
                    // traj.getFinalPose().getRotation() :
                    // traj.flipped().getFinalPose().getRotation();
                    currentTargetAngle = AllianceFlipUtil.apply(traj.getFinalPose()).getRotation();
                })
                .until(() -> Math.abs(currentTargetAngle.getDegrees() - getHeading().getDegrees()) < 4);
    }
    public Command correctHeading(Supplier<Pose2d> targetPoseSupplier) {
        return Commands.run(() -> 
        {
        swerveDrive.drive(swerveDrive.swerveController.getTargetSpeeds(0, 0, currentTargetAngle.getRadians(),
                getHeading().getRadians(), swerveDrive.getMaximumVelocity()));
        })
                .beforeStarting(() -> 
                {
                swerveDrive.setHeadingCorrection(true);
                //currentTargetAngle = (DriverStation.getAlliance().get() == Alliance.Blue) ? traj.getFinalPose().getRotation() : traj.flipped().getFinalPose().getRotation();
                currentTargetAngle = AllianceFlipUtil.apply(targetPoseSupplier.get().getRotation());
                })
                .until(() -> Math.abs(currentTargetAngle.getDegrees() - getHeading().getDegrees()) < 4); //TODO: edit tolerance
    }
    public Command correctHeading(Rotation2d targetRotation) {
        return Commands.run(() -> 
        {
        swerveDrive.drive(swerveDrive.swerveController.getTargetSpeeds(0, 0, currentTargetAngle.getRadians(),
                getHeading().getRadians(), swerveDrive.getMaximumVelocity()));
        })
                .beforeStarting(() -> 
                {
                swerveDrive.setHeadingCorrection(true);
                //currentTargetAngle = (DriverStation.getAlliance().get() == Alliance.Blue) ? traj.getFinalPose().getRotation() : traj.flipped().getFinalPose().getRotation();
                currentTargetAngle = AllianceFlipUtil.apply(targetRotation);
                })
                .until(() -> Math.abs(currentTargetAngle.getDegrees() - getHeading().getDegrees()) < 4); //TODO: edit tolerance
    }

    /**
     * Command to drive the robot using translative values and heading as a
     * setpoint.
     *
     * @param translationX Translation in the X direction. Cubed for smoother
     *                     controls.
     * @param translationY Translation in the Y direction. Cubed for smoother
     *                     controls.
     * @param headingX     Heading X to calculate angle of the joystick.
     * @param headingY     Heading Y to calculate angle of the joystick.
     * @return Drive command.
     */
    public Command headingDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier headingX,
            DoubleSupplier headingY) {
        return run(() -> {
            swerveDrive.setHeadingCorrection(true);
            double[] deadbandRotationInputs = AllDeadbands
                    .applyCircularDeadband(new double[] { headingX.getAsDouble(), headingY.getAsDouble() }, 0.95);
            double rawXInput = translationX.getAsDouble();
            double rawYInput = translationY.getAsDouble();
            double[] scaledDeadbandTranslationInputs = AllDeadbands
                    .applyScaledSquaredCircularDeadband(new double[] { rawXInput, rawYInput }, 0.1);
            double xInput = scaledDeadbandTranslationInputs[0];
            double yInput = scaledDeadbandTranslationInputs[1];
            // Make the robot move
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput * speedFactor, yInput * speedFactor,
                    deadbandRotationInputs[0],
                    deadbandRotationInputs[1],
                    swerveDrive.getYaw().getRadians(),
                    swerveDrive.getMaximumVelocity()));
        });
    }

    public Command headingDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            Supplier<Rotation2d> headingAngle) {
        return run(() -> {
            swerveDrive.setHeadingCorrection(true);
            double rawXInput = translationX.getAsDouble();
            double rawYInput = translationY.getAsDouble();
            double[] scaledDeadbandTranslationInputs = AllDeadbands
                    .applyScaledSquaredCircularDeadband(new double[] { rawXInput, rawYInput }, 0.1);
            double xInput = scaledDeadbandTranslationInputs[0];
            double yInput = scaledDeadbandTranslationInputs[1];
            // Make the robot move
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(
                    xInput * speedFactor,
                    yInput * speedFactor,
                    headingAngle.get().getDegrees(),
                    swerveDrive.getYaw().getRadians(),
                    swerveDrive.getMaximumVelocity()));
        });
    }

    /**
     * <h2>More features</h2>
     *
     * @param translationX      Translation input in the X direction.
     * @param translationY      Translation input in the Y direction.
     * @param rotationX         Rotation input in the X direction.
     * @param rotationY         Rotation input in the Y direction.
     * @param leftRotationRate  Left rotation input that overrides heading angle.
     * @param rightRotationRate Right rotation input that overrides heading angle.
     * @return A better combined drive command.
     */
    public Command enhancedHeadingDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier rotationX, DoubleSupplier rotationY,
            DoubleSupplier leftRotationRate, DoubleSupplier rightRotationRate) {
        return run(() -> {
            double[] deadbandRotationInputs = AllDeadbands
                    .applyCircularDeadband(new double[] { rotationX.getAsDouble(), rotationY.getAsDouble() }, 0.95);
            double leftRotationInput = MathUtil.applyDeadband(leftRotationRate.getAsDouble(), 0.05);
            double rightRotationInput = MathUtil.applyDeadband(rightRotationRate.getAsDouble(), 0.05);
            double rawXInput = translationX.getAsDouble();
            double rawYInput = translationY.getAsDouble();
            double[] scaledDeadbandTranslationInputs = AllDeadbands
                    .applyScaledSquaredCircularDeadband(new double[] { rawXInput, rawYInput }, 0.1);
            double xInput = scaledDeadbandTranslationInputs[0];
            double yInput = scaledDeadbandTranslationInputs[1];
            // determining if target angle is commanded
            if (deadbandRotationInputs[0] != 0 || deadbandRotationInputs[1] != 0) {
                currentTargetAngle = Rotation2d
                        .fromRadians(Math.atan2(deadbandRotationInputs[1], deadbandRotationInputs[0]));
            }
            if (leftRotationInput != 0 && rightRotationInput == 0) {
                swerveDrive.setHeadingCorrection(false);
                double leftRotationOutput = Math.pow(leftRotationInput, 3) * swerveDrive.getMaximumAngularVelocity() * 2 //For some reason max angular velocity is too low
                        * speedFactor;
                swerveDrive.drive(new Translation2d(
                        xInput * swerveDrive.getMaximumVelocity() * speedFactor,
                        yInput * swerveDrive.getMaximumVelocity() * speedFactor),
                        leftRotationOutput,
                        true, false);
                currentTargetAngle = null;
            }
            // If right trigger pressed, rotate left at a rate proportional to the right
            // trigger input
            else if (rightRotationInput != 0 && leftRotationInput == 0) {
                swerveDrive.setHeadingCorrection(false);
                double rightRotationOutput = -Math.pow(rightRotationInput, 3) * swerveDrive.getMaximumAngularVelocity()
                        * 2 // For some reason max angular velocity is too low
                        * speedFactor;
                swerveDrive.drive(new Translation2d(
                        xInput * swerveDrive.getMaximumVelocity() * speedFactor,
                        yInput * swerveDrive.getMaximumVelocity() * speedFactor),
                        rightRotationOutput,
                        true, false);
                currentTargetAngle = null;
            }
            // If no triggers are pressed or both are pressed, use the right stick for
            // heading angle steering
            else {
                // If there is no current target angle (last action was spin), then don't
                // command the angle

                swerveDrive.setHeadingCorrection(currentTargetAngle != null);
                if (currentTargetAngle != null) {
                    // Make the robot move
                    driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput * speedFactor,
                            yInput * speedFactor, currentTargetAngle.getRadians(), swerveDrive.getYaw().getRadians(),
                            swerveDrive.getMaximumVelocity()));
                } else {
                    swerveDrive.drive(new Translation2d(
                            xInput * swerveDrive.getMaximumVelocity() * speedFactor,
                            yInput * swerveDrive.getMaximumVelocity() * speedFactor),
                            0,
                            true, false);
                }
            }
        });
    }

    /**
     * Command to drive the robot using translative values and heading as angular
     * velocity.
     *
     * @param translationX     Translation in the X direction. Cubed for smoother
     *                         controls.
     * @param translationY     Translation in the Y direction. Cubed for smoother
     *                         controls.
     * @param angularRotationX Angular velocity of the robot to set. Cubed for
     *                         smoother controls.
     * @return Drive command.
     */
    public Command rotationRateDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier angularRotationX) {
        return run(() -> {
            double rawXInput = translationX.getAsDouble();
            double rawYInput = translationY.getAsDouble();
            double[] scaledDeadbandTranslationInputs = AllDeadbands
                    .applyScaledSquaredCircularDeadband(new double[] { rawXInput, rawYInput }, 0.1);
            double xInput = scaledDeadbandTranslationInputs[0];
            double yInput = scaledDeadbandTranslationInputs[1];
            double rotationRate = Math.pow(MathUtil.applyDeadband(angularRotationX.getAsDouble(), 0.1), 3);
            swerveDrive.setHeadingCorrection(false);
            // Make the robot move
            swerveDrive.drive(new Translation2d(xInput * swerveDrive.getMaximumVelocity() * speedFactor,
                    yInput * swerveDrive.getMaximumVelocity() * speedFactor),
                    Math.pow(rotationRate, 3) * swerveDrive.getMaximumAngularVelocity() * speedFactor,
                    true, false);
        });
    }

    /**
     * Command to drive the robot using translative values and heading as a
     * setpoint.
     *
     * @param translationX Translation in the X direction.
     * @param translationY Translation in the Y direction.
     * @param rotation     Rotation as a value between [-1, 1] converted to radians.
     * @return Drive command.
     */
    public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
        // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
        // correction for this kind of control.
        return run(() -> {
            // Make the robot move
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(translationX.getAsDouble(),
                    translationY.getAsDouble(),
                    rotation.getAsDouble() * Math.PI,
                    swerveDrive.getYaw().getRadians(),
                    swerveDrive.getMaximumVelocity()));
        });
    }

    /**
     * The primary method for controlling the drivebase. Takes a
     * {@link Translation2d} and a rotation rate, and
     * calculates and commands module states accordingly. Can use either open-loop
     * or closed-loop velocity control for
     * the wheel velocities. Also has field- and robot-relative modes, which affect
     * how the translation vector is used.
     *
     * @param translation   {@link Translation2d} that is the commanded linear
     *                      velocity of the robot, in meters per
     *                      second. In robot-relative mode, positive x is towards
     *                      the bow (front) and positive y is
     *                      towards port (left). In field-relative mode, positive x
     *                      is away from the alliance wall
     *                      (field North) and positive y is towards the left wall
     *                      when looking through the driver station
     *                      glass (field West).
     * @param rotation      Robot angular rate, in radians per second. CCW positive.
     *                      Unaffected by field/robot
     *                      relativity.
     * @param fieldRelative Drive mode. True for field-relative, false for
     *                      robot-relative.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation,
                rotation,
                fieldRelative,
                false); // Open loop is disabled since it shouldn't be used most of the time.
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    /**
     * Drive according to the chassis robot oriented velocity.
     *
     * @param velocity Robot oriented {@link ChassisSpeeds}
     */
    public void drive(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }

    @Override
    public void simulationPeriodic() {
    }

    /**
     * Get the swerve drive kinematics object.
     *
     * @return {@link SwerveDriveKinematics} of the swerve drive.
     */
    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not
     * need to be reset when calling this
     * method. However, if either gyro angle or module position is reset, this must
     * be called in order for odometry to
     * keep working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */
    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    /**
     * Sets the odometry angle to the current gyro angle.
     * 
     * @return A command to reset the angle offset of the odometry to zero.
     */
    public Command zeroOdometryAngleOffset() {
        return runOnce(() -> resetOdometry(new Pose2d(getPose().getTranslation(), getGyroYaw())));
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by
     * odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Post the trajectory to the field.
     *
     * @param trajectory The trajectory to post.
     */
    public void postTrajectory(Trajectory trajectory) {
        swerveDrive.postTrajectory(trajectory);
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but
     * facing toward 0.
     */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
        currentTargetAngle = new Rotation2d();
    }

    /**
     * Sets the current angle of the gyro. If the robot reaches the same angle, the
     * gyro will report this angle.
     * 
     * @param currentAngle The angle that the gyro should read in its current state.
     */
    public void setGyroAngle(Rotation2d currentAngle) {
        swerveDrive.setGyro(new Rotation3d(0, 0, currentAngle.getRadians()));
    }

    /**
     * Sets the drive motors to brake/coast mode.
     *
     * @param brake True to set motors to brake mode, false for coast.
     */
    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    /**
     * Gets the current yaw angle of the robot, as reported by the swerve pose
     * estimator in the underlying drivebase.
     * Note, this is not the raw gyro reading, this may be corrected from calls to
     * resetOdometry().
     *
     * @return The yaw angle
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Gets the current yaw angle of the robot, as reported by the gyro
     * Note, this is the raw gyro reading, this may not be corrected from calls to
     * resetOdometry().
     *
     * @return The yaw angle
     */
    public Rotation2d getGyroYaw() {
        return swerveDrive.getYaw();
    }

    public Rotation2d getCurrentTargetAngle() {
        return currentTargetAngle;
    }

    public void setTargetAngle(Rotation2d newTargetAngle) {
        currentTargetAngle = newTargetAngle;
    }

    public Command setSlowSpeed() {
        return new InstantCommand(() -> speedFactor = 0.5);
    }

    public Command setNormalSpeed() {
        return new InstantCommand(() -> speedFactor = 1);
    }

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Get the {@link SwerveController} in the swerve drive.
     *
     * @return {@link SwerveController} from the {@link SwerveDrive}.
     */
    public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }

    /**
     * Get the {@link SwerveDriveConfiguration} object.
     *
     * @return The {@link SwerveDriveConfiguration} fpr the current drive.
     */
    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    /**
     * Lock the swerve drive to prevent it from moving.
     */
    public void lock() {
        swerveDrive.lockPose();
    }

    /**
     * Gets the current pitch angle of the robot, as reported by the imu.
     *
     * @return The heading as a {@link Rotation2d} angle
     */
    public Rotation2d getPitch() {
        return swerveDrive.getPitch();
    }

    // * Adds vision measurement from vision object to swerve
    public void addVisionData(VisionData visionData) {
        Pose2d swervePose = this.getPose();
        double previousx = swervePose.getX();
        double previousy = swervePose.getY();
        Rotation2d previousTheta = swervePose.getRotation();
        if (Double.isNaN(previousy) || Double.isNaN(previousx)) {
            hadbadreading = true;
            previousx = 0;
            previousy = 0;
            System.out.println("Swerve Pose is NaN comeing in ");
        }

        if (Double.isNaN(visionData.getVisionPose().getX()) || Double.isNaN(visionData.getVisionPose().getY())) {
            System.out.println("Recived a bad vision pose");
            return;
        }

        if (hadbadreading) {
            resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d()));
            return;
        }

        swerveDrive.addVisionMeasurement(visionData.getVisionPose(), visionData.getTime(),
                visionData.getVisionReliability());
        Pose2d newPose = this.getPose();
        if (Double.isNaN(newPose.getX()) || Double.isNaN(newPose.getY())) {
            // hadbadreading = true;
            Pose2d pose = new Pose2d(previousx, previousy, previousTheta);
            swerveDrive.resetOdometry(pose);
            System.out.println("SwerveDrive is nan after vision");
        }
    }

    public boolean isvisionOk() {
        return !hadbadreading;
    }

    /**
     * Add a fake vision reading for testing purposes.
     */
    public void addFakeVisionReading() {
        swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
    }

    /**
     * Edits the drive PID with the specified P value.
     *
     * @param p The desired P value
     */
    public void configureDrivePID(double p) {
        for (SwerveModule module : swerveDrive.swerveDriveConfiguration.modules) {
            CANSparkMax driveMotor = (CANSparkMax) module.getDriveMotor().getMotor();
            driveMotor.getPIDController().setP(p);
        }
    }

    /**
     * Edits the angle PID with the specified P and D values.
     *
     * @param p The desired P value
     * @param d The desired D value
     */
    public void configureAnglePID(double p, double d) {
        for (SwerveModule module : swerveDrive.swerveDriveConfiguration.modules) {
            CANSparkMax angleMotor = (CANSparkMax) module.getAngleMotor().getMotor();
            angleMotor.getPIDController().setP(p);
            angleMotor.getPIDController().setD(d);
        }
    }

    /**
     * Edits the heading PID with the specified P and D values.
     *
     * @param p The desired P value
     * @param d The desired D value
     */
    public void configureHeadingPID(double p, double d) {
        swerveDrive.swerveController.thetaController.setPID(p, 0, d);
    }

    @Override
    public void periodic() {
        if (velocityP.getEntry().getDouble(0) != prevVelocityP) {
            prevVelocityP = velocityP.getEntry().getDouble(0);
            configureDrivePID(velocityP.getEntry().getDouble(0));
        }

        if (angleP.getEntry().getDouble(0) != prevAngleP || angleD.getEntry().getDouble(0) != prevAngleD) {
            prevAngleP = angleP.getEntry().getDouble(0);
            prevAngleD = angleD.getEntry().getDouble(0);
            configureAnglePID(angleP.getEntry().getDouble(0), angleD.getEntry().getDouble(0));
        }

        if (headingP.getEntry().getDouble(0) != prevHeadingP || headingD.getEntry().getDouble(0) != prevHeadingD) {
            prevHeadingP = headingP.getEntry().getDouble(0);
            prevHeadingD = headingD.getEntry().getDouble(0);
            configureHeadingPID(headingP.getEntry().getDouble(0), headingD.getEntry().getDouble(0));
        }

        swerveDrive.updateOdometry();

        SmartDashboard.putNumber("Vision-Swerve-PoseX", this.getPose().getX());
        SmartDashboard.putNumber("Vision-Swerve-PoseY", this.getPose().getY());
        SmartDashboard.putNumber("Vision-Swerve-PoseTheta", this.getPose().getRotation().getDegrees());
    }

    public void setSwerveOffsets() {
        Rotation2d[] currentOffsets = new Rotation2d[4];
        Rotation2d[] newOffsets = new Rotation2d[4];
        Rotation2d[] measuredPositions = new Rotation2d[4];
        AbsoluteEncoder[] encoders = new AbsoluteEncoder[4];
        for (int i = 0; i < 4; i++) {
            encoders[i] = (AbsoluteEncoder) swerveDrive.getModules()[i].getAbsoluteEncoder().getAbsoluteEncoder();
            currentOffsets[i] = Rotation2d.fromDegrees(encoders[i].getZeroOffset());
            measuredPositions[i] = Rotation2d.fromDegrees(encoders[i].getPosition());
            newOffsets[i] = currentOffsets[i].plus(measuredPositions[i])
                    .plus(Rotation2d.fromDegrees(getAngleForModule(i)));
            encoders[i].setZeroOffset(MathUtil.inputModulus(newOffsets[i].getDegrees(), 0, 360));
            swerveDrive.getModules()[i].getAngleMotor().burnFlash();
        }
    }

    private double getAngleForModule(int moduleNumber) {
        return switch (moduleNumber) {
            case 0 -> -90;
            case 1 -> 0;
            case 2 -> -180;
            case 3 -> -270;
            default -> 0;
        };
    }

    private String getPIDVals() {
        String temp = "";
        for (SwerveModule module : swerveDrive.swerveDriveConfiguration.modules) {
            temp += "P: " + (CANSparkMax) module.getDriveMotor().getMotor() + ", I: "
                    + module.configuration.velocityPIDF.i + ", D: " +
                    module.configuration.velocityPIDF.d + "\n";
        }
        return temp.strip();
    }

    // Vision Stuff

    // Takes a point and returns the desired heading for the swerve to be pointing
    // at the given point using the curent pose
    private double getAngleToPoint(Pose2d targetPoint) {
        Pose2d currentPose = this.getPose();
        double desired_heading_rad = Math.atan2(targetPoint.getY() - currentPose.getY(),
                targetPoint.getX() - currentPose.getX());
        return desired_heading_rad;
    }

    // needs tp be called repeadatly
    public Command pointAtVisionTarget(Pose2d targetPoint) {
        return new InstantCommand(() -> {
            double desired_heading_deg = getAngleToPoint(targetPoint);
            Rotation2d desired_heading = Rotation2d.fromDegrees(desired_heading_deg);
            swerveDrive.setHeadingCorrection(true);
            setTargetAngle(desired_heading);
        });
    }

    /**
     * <h2>Vision Targeting</h2>
     * Drives field oriented with translation X, Y, and points at the given target
     * point
     * 
     * @param translationX Supplier of translation in X axis
     * @param translationY Supplier of translation in Y axis
     * @param targetPoint  Supplier of target point
     * @return A RunCommand that drives the swerve drive with given translation and
     *         rotation
     */
    public Command driveTranslationAndPointAtTarget(DoubleSupplier translationX, DoubleSupplier translationY,
            Pose2d targetPoint) {
        return run(() -> {
            // Pose2d flippedTargetPoint = AllianceFlipUtil.apply(targetPoint);
            DriverStation.Alliance alliance = DriverStation.getAlliance().isPresent()
                    ? DriverStation.getAlliance().get()
                    : DriverStation.Alliance.Blue;
            double desiredHeadingRad = getAngleToPoint(
                    FieldConstants.allianceFlipper(new Pose3d(targetPoint), alliance).toPose2d());
            Rotation2d desired_heading = Rotation2d.fromRadians(desiredHeadingRad);
            swerveDrive.setHeadingCorrection(true);
            double rawXInput = translationX.getAsDouble();
            double rawYInput = translationY.getAsDouble();
            double[] scaledDeadbandTranslationInputs = AllDeadbands
                    .applyScaledSquaredCircularDeadband(new double[] { rawXInput, rawYInput }, 0.1);
            double xInput = scaledDeadbandTranslationInputs[0];
            double yInput = scaledDeadbandTranslationInputs[1];
            // Make the robot move
            setTargetAngle(desired_heading);
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(
                    xInput * speedFactor,
                    yInput * speedFactor,
                    desiredHeadingRad,
                    swerveDrive.getYaw().getRadians(),
                    swerveDrive.getMaximumVelocity()));
        });
    }
}
