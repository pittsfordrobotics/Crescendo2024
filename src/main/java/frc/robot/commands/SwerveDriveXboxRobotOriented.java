// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.Swerve;

public class SwerveDriveXboxRobotOriented extends Command {

  private Swerve swerveDrive;
  private CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private double xAxis;
  private double yAxis;
  private double rotateRate;

  /** Swerve Drive with robot oriented steering. */
  public SwerveDriveXboxRobotOriented(Swerve swerveDrive) {
    addRequirements(swerveDrive);
    this.swerveDrive = swerveDrive;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      xAxis = MathUtil.applyDeadband(-driverController.getLeftY(), SwerveConstants.driverControllerLeftDeadband); // update all controller inputs, Xbox controller has different X and Y directions
      yAxis = MathUtil.applyDeadband(-driverController.getLeftX(), SwerveConstants.driverControllerLeftDeadband);
      rotateRate = MathUtil.applyDeadband(-driverController.getRightX(), 0.1);
      swerveDrive.updateSwerveModuleStates(xAxis, yAxis, rotateRate);
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
