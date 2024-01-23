// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.SwerveOffsets;
import frc.robot.subsystems.swerve.Swerve;

public class SwerveSetZeroOffsets extends Command {
  private Swerve swerveDrive;

  /**
   * Saves the zero offsets of all swerve modules as their current angle.
   * Only call this command when using swerve alignment tool.
   */
  public SwerveSetZeroOffsets(Swerve swerveDrive) {
    addRequirements(swerveDrive);
    this.swerveDrive = swerveDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d[] offsetModuleAngles = swerveDrive.getModuleAngles();
    Rotation2d[] currentOffsets = swerveDrive.getModuleZeroOffsets();

    Rotation2d calculatedFLOffset = offsetModuleAngles[0].plus(currentOffsets[0]);
    Rotation2d calculatedFROffset = offsetModuleAngles[1].plus(currentOffsets[1]);
    Rotation2d calculatedBLOffset = offsetModuleAngles[2].plus(currentOffsets[2]);
    Rotation2d calculatedBROffset = offsetModuleAngles[3].plus(currentOffsets[3]);

    System.out.println("Calculated FL offset: " + calculatedFLOffset.getDegrees());
    System.out.println("Calculated FR offset: " + calculatedFROffset.getDegrees());
    System.out.println("Calculated BL offset: " + calculatedBLOffset.getDegrees());
    System.out.println("Calculated BR offset: " + calculatedBROffset.getDegrees());

    SwerveOffsets offsets = new SwerveOffsets();
    offsets.FLOffset = calculatedFLOffset;
    offsets.FROffset = calculatedFROffset;
    offsets.BLOffset = calculatedBLOffset;
    offsets.BROffset = calculatedBROffset;

    offsets.saveToConfigFile();

    // swerveDrive.updateSwerveOffsets();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
