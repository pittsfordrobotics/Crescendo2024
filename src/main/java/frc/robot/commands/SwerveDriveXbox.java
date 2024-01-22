// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.Swerve;

public class SwerveDriveXbox extends Command {
  private Swerve swerveDrive;
  private CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private double rotateX;
  private double rotateY;
  private double lastRotateX;
  private double lastRotateY;
  /** Creates a new SwerveDriveXbox. */
  public SwerveDriveXbox(Swerve swerveDrive) {
    addRequirements(swerveDrive);
    this.swerveDrive = swerveDrive;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // swerveDrive.zeroGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  /* (non-Javadoc)
   * @see edu.wpi.first.wpilibj2.command.Command#execute()
   */
  @Override
  public void execute() {
    double xAxis = MathUtil.applyDeadband(-driverController.getLeftY(), SwerveConstants.driverControllerLeftDeadband); // update all controller inputs, Xbox controller has different X and Y directions
    double yAxis = MathUtil.applyDeadband(-driverController.getLeftX(), SwerveConstants.driverControllerLeftDeadband);
    rotateX = -driverController.getRightY();
    rotateY = -driverController.getRightX();
    double radius = MathUtil.applyDeadband(Math.sqrt(Math.pow(rotateX, 2) + Math.pow(rotateY, 2)), SwerveConstants.driverControllerRightDeadband);
    if(radius == 0){ //If in deadband, don't update the target direction
      swerveDrive.updateSwerveModuleStates(xAxis, yAxis);
    }else{
      Rotation2d targetAngle = Rotation2d.fromRadians(Math.atan2(rotateY, rotateX));
      swerveDrive.updateSwerveModuleStates(xAxis, yAxis, targetAngle);
    }

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
