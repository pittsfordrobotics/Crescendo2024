// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToPoint_Test extends InstantCommand {
  private SwerveSubsystem swerveSubsystem;

  public DriveToPoint_Test() {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        // adds 3 ft to the x of the apriltas pose
    Transform2d offset = new Transform2d(3,0, null);
    Pose2d targetpose = FieldConstants.aprilTags.getTags().get(0).pose.toPose2d().plus(null).plus(offset);
    swerveSubsystem.driveToPose(targetpose);
  }
}
