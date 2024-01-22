// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.swerve.Swerve;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Use the specified choreo trajectory in auto */
  public static Command choreoSwerveAuto(Swerve swerveDrive, String trajName) {
    ChoreoTrajectory traj = Choreo.getTrajectory(trajName);
    return Commands.sequence(new SwerveDriveChoreo(swerveDrive, traj));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
