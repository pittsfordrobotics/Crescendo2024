// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.DisabledInstantCommand;
import frc.robot.commands.SwerveDriveXbox;
import frc.robot.commands.SwerveSetZeroOffsets;
import frc.robot.lib.BetterSwerveModuleState;
import frc.robot.lib.SecondOrderKinematics;
import frc.robot.lib.SwerveOffsets;
import frc.robot.lib.SwerveOptimizer;
import edu.wpi.first.math.controller.PIDController;

public class Swerve extends SubsystemBase {

  // Defining swerve modules
  private SwerveModuleIO moduleFL;
  private SwerveModuleIO moduleFR;
  private SwerveModuleIO moduleBL;
  private SwerveModuleIO moduleBR;

  private final SwerveModuleIO[] moduleIO;
  private final BetterSwerveModuleState[] lastModuleStates = new BetterSwerveModuleState[4];
  private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
  private ChassisSpeeds actualFieldRelativeSpeeds;
  private ChassisSpeeds actualRobotRelativeChassisSpeeds;
  private ChassisSpeeds targetFieldRelativeSpeeds;
  private ChassisSpeeds targetRobotRelativeChassisSpeeds;
  private Pose2d pose;
  //private final SwerveDrivePoseEstimator poseEstimator;

  // Initialize a PID controller for calculating the wanted angular velocity based on the desired angle
  private PIDController SwerveTargetAnglePID = new PIDController(SwerveConstants.ROBOT_ANG_P, SwerveConstants.ROBOT_ANG_I, SwerveConstants.ROBOT_ANG_D);
  // private BetterSwerveModuleState[] wantedModuleStates = new BetterSwerveModuleState[4];
  private SwerveModuleState[] wantedModuleStates = new SwerveModuleState[4];
  private BetterSwerveModuleState[] actualStates = new BetterSwerveModuleState[4];
  // private final SecondOrderKinematics kinematics = SwerveConstants.BETTER_DRIVE_KINEMATICS;
  private final SwerveDriveKinematics kinematics = SwerveConstants.DRIVE_KINEMATICS;
  private final SwerveDriveOdometry odometry;
  //private final SwerveOffsets moduleOffsets;
  //Rotation2d[] offsets;

  private Rotation2d robotRelativeAngle = new Rotation2d();
  private Rotation2d targetAngle = new Rotation2d();
  private Pigeon2 pigeon= new Pigeon2(SwerveConstants.CAN_PIGEON);

  /** Creates a new Swerve Drive object with 4 modules specified by SwerveConstants */
  public Swerve() {
    this.zeroGyro();

    SwerveOffsets offsets = SwerveOffsets.readFromConfig();

    // Add angles of offset based on mounting angle of modules
    // Add in later?
    Rotation2d flOffset = offsets.FLOffset.plus(Rotation2d.fromDegrees(-90));
    Rotation2d frOffset = offsets.FROffset.plus(Rotation2d.fromDegrees(0));
    Rotation2d blOffset = offsets.BLOffset.plus(Rotation2d.fromDegrees(-180));
    Rotation2d brOffset = offsets.BROffset.plus(Rotation2d.fromDegrees(-270));
    
    moduleFL = new SwerveModuleIO(SwerveConstants.CAN_FL_DRIVE, SwerveConstants.CAN_FL_STEER, flOffset);
    moduleFR = new SwerveModuleIO(SwerveConstants.CAN_FR_DRIVE, SwerveConstants.CAN_FR_STEER, frOffset);
    moduleBL = new SwerveModuleIO(SwerveConstants.CAN_BL_DRIVE, SwerveConstants.CAN_BL_STEER, blOffset);
    moduleBR = new SwerveModuleIO(SwerveConstants.CAN_BR_DRIVE, SwerveConstants.CAN_BR_STEER, brOffset);

    SmartDashboard.putNumber("FL_PURE_OFFSET", offsets.FLOffset.getDegrees());
    SmartDashboard.putNumber("FR_PURE_OFFSET", offsets.FROffset.getDegrees());
    SmartDashboard.putNumber("BL_PURE_OFFSET", offsets.BLOffset.getDegrees());
    SmartDashboard.putNumber("BR_PURE_OFFSET", offsets.BROffset.getDegrees());

    moduleIO = new SwerveModuleIO[]{moduleFL, moduleFR, moduleBL, moduleBR}; // initializes motors and encoders for all 4 swerve modules.
    for (int i = 0; i < 4; i++) {
      moduleIO[i].updateInputs();
      lastModuleStates[i] = new BetterSwerveModuleState();
      modulePositions[i] = new SwerveModulePosition();
    }
    
    odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), modulePositions);
    // TODO: figure out what the pose estimator is used for.
    // poseEstimator = new SwerveDrivePoseEstimator(kinematics, getRobotRelativeAngle(), modulePositions, new Pose2d(), VecBuilder.fill(0.003, 0.003, 0.0002), VecBuilder.fill(0.9, 0.9, 0.9));
    Shuffleboard.getTab("CONFIG").add("Record Wheel Offsets", new SwerveSetZeroOffsets(this));
    Shuffleboard.getTab("CONFIG").add("Toggle Brake-Coast", new DisabledInstantCommand(this::toggleCoast, this));
  }
  @Override
  public void periodic() {
    // SmartDashboard.putNumber("FL_OFFSET", offsets[0].getDegrees());
    // SmartDashboard.putNumber("FR_OFFSET", offsets[1].getDegrees());
    // SmartDashboard.putNumber("BL_OFFSET", offsets[2].getDegrees());
    // SmartDashboard.putNumber("BR_OFFSET", offsets[3].getDegrees());
    this.getModuleAngles();
    // This method will be called once per scheduler run
  }
   /**Gets the robot's current orientation. Returns the CCW+ angle in a Rotation2d object. */
  private Rotation2d getRobotRelativeAngle(){
    double robotRelativeAngleDeg = pigeon.getYaw().getValueAsDouble();
    
    return Rotation2d.fromRadians(MathUtil.angleModulus(Math.toRadians(robotRelativeAngleDeg)));
  }
/** Drives Swerve Drive using field relative translation and rotation rate of robot.
 *  <p>Updates swerve module inputs for all swerve modules.</p> 
 *  <p>Performs inverse kinematics to create target swerve module states from controller inputs</p>
 *  <p>All controller inputs should be between -1 and 1</p>
 * @param xAxis Translation x axis input (forward+)
 * @param yAxis Translation y axis input (left+)
 * @param omega Rotation RATE input (ccw+)
 */
  public void updateSwerveModuleStates(double xAxis, double yAxis, double omega) {
    double targetSpeedX = xAxis * SwerveConstants.MAX_LINEAR_VELOCITY_METERS_PER_SECOND;
    double targetSpeedY = yAxis * SwerveConstants.MAX_LINEAR_VELOCITY_METERS_PER_SECOND;

    for (int i = 0; i < 4; i++) {
      moduleIO[i].updateInputs();
      actualStates[i] = new BetterSwerveModuleState(moduleIO[i].driveVelocityMetersPerSec, Rotation2d.fromRadians(moduleIO[i].steerAbsolutePositionRad), moduleIO[i].steerAbsoluteVelocityRadPerSec);
      modulePositions[i] = new SwerveModulePosition(actualStates[i].speedMetersPerSecond * 0.02, actualStates[i].angle);
    }
    robotRelativeAngle = getRobotRelativeAngle();
    odometry.update(robotRelativeAngle, modulePositions);
    pose = odometry.getPoseMeters();
    double targetAngularVelocity = omega * SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    actualRobotRelativeChassisSpeeds = kinematics.toChassisSpeeds(actualStates);
    actualFieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(actualRobotRelativeChassisSpeeds, new Rotation2d().minus(robotRelativeAngle)); //Find actual field relative speeds
    targetFieldRelativeSpeeds = new ChassisSpeeds(targetSpeedX, targetSpeedY, targetAngularVelocity);
    targetRobotRelativeChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(targetFieldRelativeSpeeds, robotRelativeAngle); // Convert target field relative speeds into chassis speeds
    wantedModuleStates = kinematics.toSwerveModuleStates(targetRobotRelativeChassisSpeeds);
    for (int i = 0; i < 4; i++) {
      // getCurrentAngleDeg calls updateInputs, which is a bit inefficient.
      Rotation2d currentModuleAngle = Rotation2d.fromDegrees(moduleIO[i].getCurrentAngleDeg());
      wantedModuleStates[i] = SwerveOptimizer.optimize(wantedModuleStates[i], currentModuleAngle);
    }
  }
  /** Drives Swerve Drive using field relative translation and heading angle of robot.
   *  <p>Updates swerve module inputs for all swerve modules.</p> 
   *  <p>Performs inverse kinematics to create target swerve module states from controller inputs</p>
   *  <p>All controller inputs should be between -1 and 1</p>
   * @param xAxis Translation x-axis input (left stick to left)
   * @param yAxis Translation y-axis input (left stick up)
   * @param rotateX Rotation x-axis input (right stick left)
   * @param rotateY Rotation y-axis input (right stick up)
  */
  public void updateSwerveModuleStates(double xAxis, double yAxis, Rotation2d targetAngleRad) {  
    this.targetAngle = targetAngleRad;
    updateSwerveModuleStates(xAxis, yAxis);
  };

  public void updateSwerveModuleStates(double xAxis, double yAxis) {
    // Set X and Y speeds based on max motor RPM.
    double targetSpeedX = xAxis * SwerveConstants.MAX_LINEAR_VELOCITY_METERS_PER_SECOND;
    double targetSpeedY = yAxis * SwerveConstants.MAX_LINEAR_VELOCITY_METERS_PER_SECOND;

    for (int i = 0; i < 4; i++) {
      moduleIO[i].updateInputs();
      actualStates[i] = new BetterSwerveModuleState(moduleIO[i].driveVelocityMetersPerSec, Rotation2d.fromRadians(moduleIO[i].steerAbsolutePositionRad), moduleIO[i].steerAbsoluteVelocityRadPerSec);
      modulePositions[i] = new SwerveModulePosition(actualStates[i].speedMetersPerSecond * 0.02, actualStates[i].angle);
    }
    robotRelativeAngle = getRobotRelativeAngle();
    odometry.update(robotRelativeAngle, modulePositions);
    pose = odometry.getPoseMeters();

    // New code
    // Decides how to calculate the target angular velocity based on the controller inputs (2 variable booleons and a constant boolean)
    double targetAngularVelocity = 0;
    // Logging
    SmartDashboard.putNumber("Target Angular Velocity", targetAngularVelocity);
    SmartDashboard.putNumber("Target Angle", targetAngle.getDegrees());
    SmartDashboard.putNumber("Robot Relative Angle", robotRelativeAngle.getDegrees());
    SmartDashboard.putNumber("targetSpeedX", targetSpeedX);
    SmartDashboard.putNumber("targetSpeedY", targetSpeedY);
    
    // // Just RightJoystick Code but with PID
    SwerveTargetAnglePID.enableContinuousInput(-Math.PI, Math.PI);
    SwerveTargetAnglePID.setTolerance(1/120);
    targetAngularVelocity = SwerveTargetAnglePID.calculate(robotRelativeAngle.getRadians(), targetAngle.getRadians()); // Target angular velocity is a linear function of input, with a steep slope.
    targetAngularVelocity = MathUtil.clamp(targetAngularVelocity, -SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND); // Clamp the angular velocity at the max allowed value.

    actualRobotRelativeChassisSpeeds = kinematics.toChassisSpeeds(actualStates);
    actualFieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(actualRobotRelativeChassisSpeeds, new Rotation2d().minus(robotRelativeAngle)); //Find actual field relative speeds
    targetFieldRelativeSpeeds = new ChassisSpeeds(targetSpeedX, targetSpeedY, targetAngularVelocity);
    targetRobotRelativeChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(targetFieldRelativeSpeeds, robotRelativeAngle); // Convert target field relative speeds into chassis speeds
    // wantedModuleStates = kinematics.toSwerveModuleStates(targetRobotRelativeChassisSpeeds); // Use inverse kinematics to get target swerve module states.
    wantedModuleStates = kinematics.toSwerveModuleStates(targetRobotRelativeChassisSpeeds);
    for (int i = 0; i < 4; i++) {
      // getCurrentAngleDeg calls updateInputs, which is a bit inefficient.
      Rotation2d currentModuleAngle = Rotation2d.fromDegrees(moduleIO[i].getCurrentAngleDeg());
      wantedModuleStates[i] = SwerveOptimizer.optimize(wantedModuleStates[i], currentModuleAngle);
    }
    
  };
  
  /** Drive with ChassisSpeeds input */
  public void updateSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
    for (int i = 0; i < 4; i++) {
      moduleIO[i].updateInputs();
      actualStates[i] = new BetterSwerveModuleState(moduleIO[i].driveVelocityMetersPerSec, Rotation2d.fromRadians(moduleIO[i].steerAbsolutePositionRad), moduleIO[i].steerAbsoluteVelocityRadPerSec);
      modulePositions[i] = new SwerveModulePosition(actualStates[i].speedMetersPerSecond * 0.02, actualStates[i].angle);
    }
    robotRelativeAngle = getRobotRelativeAngle();
    odometry.update(robotRelativeAngle, modulePositions);
    pose = odometry.getPoseMeters();
    //forward kinematics for odometry
    actualRobotRelativeChassisSpeeds = kinematics.toChassisSpeeds(actualStates);
    actualFieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(actualRobotRelativeChassisSpeeds, new Rotation2d().minus(robotRelativeAngle)); //Find actual field relative speeds
    targetRobotRelativeChassisSpeeds = chassisSpeeds; // Input is chassis speeds so there is no conversion
    wantedModuleStates = kinematics.toSwerveModuleStates(targetRobotRelativeChassisSpeeds); // Use inverse kinematics to get target swerve module states.
    SecondOrderKinematics.desaturateWheelSpeeds(wantedModuleStates, SwerveConstants.MAX_LINEAR_VELOCITY_METERS_PER_SECOND);
    for (int i = 0; i < 4; i++) {
      SwerveModuleState temp = BetterSwerveModuleState.optimize(wantedModuleStates[i], actualStates[i].angle);
      wantedModuleStates[i] = new BetterSwerveModuleState(temp.speedMetersPerSecond, temp.angle, actualStates[i].omegaRadPerSecond);
    }

  }

  /** Used for testing alignment, stops all modules and points them forward */
  public void driveZeroOffset() {
    for (int i = 0; i < 4; i++) {
      wantedModuleStates[i] = new BetterSwerveModuleState(0, new Rotation2d(), 0);
      moduleIO[i].drive(wantedModuleStates[i], false);
    }
  }
  /** Drive all swerve modules */
  public void drive() {
    for (int i = 0; i < 4; i++) {
      moduleIO[i].drive(wantedModuleStates[i], false);
    }
  }
  /** Reset the pigeon angle */
  public void zeroGyro() {
    pigeon.setYaw(0);
  }
  /** Get robot pose */
   public Pose2d getRobotPose() {
     return pose;
   }
  /** Get angles of swerve modules */
  public Rotation2d[] getModuleAngles() {
    Rotation2d[] moduleAngles = new Rotation2d[4];
    for(int i = 0; i < 4; i++) {
      // 'getCurrentAngleDeg()' will call updateInputs internally.
      //moduleIO[i].updateInputs();
      moduleAngles[i] = Rotation2d.fromDegrees(moduleIO[i].getCurrentAngleDeg());
      // System.out.println(moduleAngles[i].getDegrees());
      SmartDashboard.putNumber("moduleAngles" + i, moduleAngles[i].getDegrees());
    }
    return moduleAngles;
  }

  public Rotation2d[] getModuleZeroOffsets() {
    Rotation2d[] currentOffsets = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      currentOffsets[i] = Rotation2d.fromRadians(moduleIO[i].getZeroOffsetRadians());
    }

    return currentOffsets;
  }

  public void resetSwerveOffsets() {
    for(int i = 0; i < 4; i++) {
      moduleIO[i].setZeroOffset(Rotation2d.fromDegrees(0));
    }
  }
  // public void updateSwerveOffsets() {
  //   for(int i = 0; i < 4; i++) {
  //     moduleIO[i].setZeroOffset(moduleOffsets.getSwerveOffsets()[i].getRadians());
  //   }
  // }
  public void toggleCoast() {
    for(SwerveModuleIO module: moduleIO) {
      module.toggleSteerBrakeMode();
    }
  }
}
