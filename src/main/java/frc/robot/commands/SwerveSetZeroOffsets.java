// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.File;
import java.util.HashMap;
import java.util.Map;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveModuleIO;

public class SwerveSetZeroOffsets extends Command {

  /** Saves the zero offsets of all swerve modules as their current angle.
   *  Only call this command when using swerve alignment tool.
   */
  public SwerveSetZeroOffsets(Swerve swerveDrive) {
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Open the JSON file containing swerve module offsets
    ObjectMapper objectMapper = new ObjectMapper();
    File swerveOffsetsFile = new File("/temp/SwerveOffsets.json");
    Map<String, Double> swerveOffsetsMap = new HashMap<String, Double>();
    

    //Create SwerveModuleIO objects to measure the current angle of the swerve modules
    SwerveModuleIO moduleFL = new SwerveModuleIO(SwerveConstants.CAN_FL_DRIVE, SwerveConstants.CAN_FL_STEER, new Rotation2d());
    SwerveModuleIO moduleFR = new SwerveModuleIO(SwerveConstants.CAN_FR_DRIVE, SwerveConstants.CAN_FR_STEER, new Rotation2d());
    SwerveModuleIO moduleBL = new SwerveModuleIO(SwerveConstants.CAN_BL_DRIVE, SwerveConstants.CAN_BL_STEER, new Rotation2d());
    SwerveModuleIO moduleBR = new SwerveModuleIO(SwerveConstants.CAN_BR_DRIVE, SwerveConstants.CAN_BR_STEER, new Rotation2d());
    //Check the angle of modules and put them in the hashmap
    swerveOffsetsMap.put("FL_PURE_OFFSET", moduleFL.getCurrentAngleDeg());
    swerveOffsetsMap.put("FR_PURE_OFFSET", moduleFR.getCurrentAngleDeg());
    swerveOffsetsMap.put("BL_PURE_OFFSET", moduleBL.getCurrentAngleDeg());
    swerveOffsetsMap.put("BR_PURE_OFFSET", moduleBR.getCurrentAngleDeg());
    //Save these values in the JSON file
    try {
      objectMapper.writeValue(swerveOffsetsFile, new TypeReference<Map<String, Double>>(){});
    } catch (Exception e) {
      e.printStackTrace();
    }
  }
  @Override
  public boolean runsWhenDisabled(){
    return true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
