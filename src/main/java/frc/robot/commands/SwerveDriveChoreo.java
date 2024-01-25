// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

public class SwerveDriveChoreo extends Command {
  private Swerve swerveDrive;
  private ChoreoTrajectory trajectory;
  private Command swerveCommand;

  /** Drive the swerve drive with the selected choreo trajectory */
  public SwerveDriveChoreo(Swerve swerveDrive, ChoreoTrajectory trajectory) {
    addRequirements(swerveDrive);
    this.swerveDrive = swerveDrive;
    this.trajectory = trajectory;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ChoreoControlFunction swerveController = Choreo.choreoSwerveController(new PIDController(5, 0, 0), new PIDController(5, 0, 0), new PIDController(2, 0, 0));
    swerveCommand = Choreo.choreoSwerveCommand(
      trajectory, 
      swerveDrive::getRobotPose, 
      swerveController, 
      swerveDrive::updateSwerveModuleStates, // Update Swerve Module States with chassis speeds, 
      () -> false,  // True if we need the field to be flipped. TODO make it true for red, false for blue
      swerveDrive);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveCommand.execute();
    swerveDrive.drive();
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
