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
    // Set the offsets to zero to get the raw values
    swerveDrive.resetSwerveOffsets();
    
    // Read the new offsets and save them.
    Rotation2d[] moduleAngles = swerveDrive.getModuleAngles();
    SwerveOffsets offsets = new SwerveOffsets();
    offsets.FLOffset = moduleAngles[0];
    offsets.FROffset = moduleAngles[1];
    offsets.BLOffset = moduleAngles[2];
    offsets.BROffset = moduleAngles[3];

    offsets.saveToConfigFile();

    // Display these values to the SmartDashboard
    ShuffleboardTab configTab = Shuffleboard.getTab("CONFIG"); // Creating the smartdashboard tab if not already created
    configTab.addDoubleArray("Swerve Offsets", () -> new double[] {
        offsets.FLOffset.getDegrees(),
        offsets.FROffset.getDegrees(),
        offsets.BROffset.getDegrees(),
        offsets.BLOffset.getDegrees()
    }); 

    // Set the new offsets
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
