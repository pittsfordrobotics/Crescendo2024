// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.Swerve;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

// This command used the swerve subsystem to zero the gyro when called
public class ZeroGyro extends InstantCommand {
  private Swerve swerveDrive;
  public ZeroGyro(Swerve swerveDrive) {
    addRequirements(swerveDrive);
    this.swerveDrive = swerveDrive;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveDrive.zeroGyro();
    swerveDrive.updateSwerveModuleStates(0, 0, new Rotation2d());
  }
}
