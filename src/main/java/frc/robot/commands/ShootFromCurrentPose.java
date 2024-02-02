// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class ShootFromCurrentPose extends Command {
  private Shooter shooter;
  private Intake intake;
  private SwerveSubsystem swerveDrive;
  private Pose2d currentPose;
  private Pose2d speakerPose;
  private double speakerDistanceMeters;
  private double speakerHeight;
  private double targetShooterAngleRad;
  private BooleanSupplier shootSupplier;
  private boolean shoot;
  /**Auto-adjusts the shooter angle based on distance to the speaker and height of speaker
   * @param shooter The shooter object (required)
   * @param intake The intake object (required)
   * @param swerveDrive Swerve drive object for current pose
   * @param shootSupplier Boolean input for whether to shoot
   */
  public ShootFromCurrentPose(Shooter shooter, Intake intake, SwerveSubsystem swerveDrive, BooleanSupplier shootSupplier) {
    addRequirements(shooter, intake);
    this.shooter = shooter;
    this.intake = intake;
    this.swerveDrive = swerveDrive;
    this.shootSupplier = shootSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speakerPose = FieldConstants.Speaker.centerSpeakerOpening;
    speakerHeight = FieldConstants.topRightSpeaker.getZ();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPose = swerveDrive.getPose();
    // Get distance to the speaker in 2D.
    speakerDistanceMeters = Math.sqrt(Math.pow(speakerPose.minus(currentPose).getX(), 2) + Math.pow(speakerPose.minus(currentPose).getY(), 2));
    targetShooterAngleRad = Math.atan2(speakerHeight, speakerDistanceMeters);
    // Set the PID setpoints for shooter/intake position
    shooter.setShooterPivotangle(Math.toDegrees(targetShooterAngleRad));
    intake.setIntakePivotAngle(180);
    shooter.driveShooter(1);
    shoot = shootSupplier.getAsBoolean(); //Poll for whether to shoot
    if(shoot && shooter.getShooterRpm() >= 5000) { // Shoot the shooter if requested and shooter is up to speed
      shooter.setIndexer(1);
    } else { // Stop the indexer otherwise
      shooter.setIndexer(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setIndexer(0);
    shooter.driveShooter(0);
    shooter.setShooterPivotangle(0);
    intake.setIntakePivotAngle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
