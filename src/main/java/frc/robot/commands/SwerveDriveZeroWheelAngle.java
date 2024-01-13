// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

public class SwerveDriveZeroWheelAngle extends Command {
  private Swerve swerveDrive;
  /** A diagnostic command that puts all the swerve modules at 0 speed and angle. */
  public SwerveDriveZeroWheelAngle(Swerve swerveDrive) {
    addRequirements(swerveDrive);
    this.swerveDrive = swerveDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveDrive.zeroGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDrive.driveZeroOffset();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
