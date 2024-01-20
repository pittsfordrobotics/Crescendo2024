// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.File;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.Swerve;

public class SwerveSetZeroOffsets extends Command {
  private Swerve swerveDrive;

  /** Saves the zero offsets of all swerve modules as their current angle.
   *  Only call this command when using swerve alignment tool.
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
  public boolean runsWhenDisabled(){
    return true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Open the JSON file containing swerve module offsets
    ObjectMapper objectMapper = new ObjectMapper();
    File swerveOffsetsFile = new File("/home/lvuser/SwerveOffsets.json");
    Map<String, Double> swerveOffsetsMap = new HashMap<String, Double>();
    

    //Create SwerveModuleIO objects to measure the current angle of the swerve modules
    swerveDrive.resetSwerveOffsets();
    Rotation2d[] moduleAngles = swerveDrive.getModuleAngles();
    //Check the angle of modules and put them in the hashmap
    swerveOffsetsMap.put("FL_PURE_OFFSET", moduleAngles[0].getDegrees());
    swerveOffsetsMap.put("FR_PURE_OFFSET", moduleAngles[1].getDegrees());
    swerveOffsetsMap.put("BL_PURE_OFFSET", moduleAngles[2].getDegrees());
    swerveOffsetsMap.put("BR_PURE_OFFSET", moduleAngles[3].getDegrees());

    //Save these values to the SmartDashboard
    ShuffleboardTab configTab = Shuffleboard.getTab("CONFIG"); //Creating the smartdashboard tab if not already created
    // Supplier<double[]> offsetSupplier = new Supplier<double[]>() {
    //   @Override
    //   public double[] get(){
    //     double[] doubleArray = new double[4];
    //     doubleArray[0] = swerveOffsetsMap.get("FL_PURE_OFFSET");
    //     doubleArray[1] = swerveOffsetsMap.get("FR_PURE_OFFSET");
    //     doubleArray[2] = swerveOffsetsMap.get("BL_PURE_OFFSET");
    //     doubleArray[3] = swerveOffsetsMap.get("BR_PURE_OFFSET");
    //     return doubleArray;
    //   }
    // };
    configTab.addDoubleArray("Swerve Offsets", () -> new double[]{
        swerveOffsetsMap.get("FL_PURE_OFFSET"),
        swerveOffsetsMap.get("FR_PURE_OFFSET"),
        swerveOffsetsMap.get("BL_PURE_OFFSET"),
        swerveOffsetsMap.get("BR_PURE_OFFSET")
      }
    ); //Adding them to the smartdashboard tab

    //Save these values in the JSON file
    try {
      System.out.println("Checking file");
      System.out.println(swerveOffsetsFile.getAbsolutePath());
      if(swerveOffsetsFile.exists()) {
        System.out.println("Deleting file");
        swerveOffsetsFile.delete();
      }
      System.out.println("Creating file");
      swerveOffsetsFile.createNewFile();
      System.out.println("Writing values");
      objectMapper.writeValue(swerveOffsetsFile, swerveOffsetsMap);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
