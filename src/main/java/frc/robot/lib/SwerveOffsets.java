// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class SwerveOffsets {
    public Rotation2d BLOffset = new Rotation2d();
    public Rotation2d BROffset = new Rotation2d();
    public Rotation2d FLOffset = new Rotation2d();
    public Rotation2d FROffset = new Rotation2d();

    private static final String OffsetConfigFile = "/home/lvuser/SwerveOffsets.json";

    public SwerveOffsets() {
    }

    public static SwerveOffsets readFromConfig() {
        Map<String, Double> swerveOffsetsMap = new HashMap<String, Double>();
        ObjectMapper objectMapper = new ObjectMapper();
        File swerveOffsetsFile = new File(OffsetConfigFile);
        if (!swerveOffsetsFile.exists()) {
            try {
                System.out.println("Creating new file");
                swerveOffsetsFile.createNewFile();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        try {
            System.out.println("Reading file");
            swerveOffsetsMap = objectMapper.readValue(swerveOffsetsFile, new TypeReference<Map<String, Double>>() {
            });
        } catch (Exception e) {
            e.printStackTrace();
        }

        System.out.println(swerveOffsetsMap.get("FL_PURE_OFFSET"));
        System.out.println(swerveOffsetsMap.get("FR_PURE_OFFSET"));
        System.out.println(swerveOffsetsMap.get("BL_PURE_OFFSET"));
        System.out.println(swerveOffsetsMap.get("BR_PURE_OFFSET"));

        // Measured module angles when using alignment tool
        Rotation2d FL_PURE_OFFSET = Rotation2d.fromDegrees(swerveOffsetsMap.getOrDefault("FL_PURE_OFFSET", 0.0));
        Rotation2d FR_PURE_OFFSET = Rotation2d.fromDegrees(swerveOffsetsMap.getOrDefault("FR_PURE_OFFSET", 0.0));
        Rotation2d BL_PURE_OFFSET = Rotation2d.fromDegrees(swerveOffsetsMap.getOrDefault("BL_PURE_OFFSET", 0.0));
        Rotation2d BR_PURE_OFFSET = Rotation2d.fromDegrees(swerveOffsetsMap.getOrDefault("BR_PURE_OFFSET", 0.0));

        SwerveOffsets offsets = new SwerveOffsets();
        offsets.FLOffset = FL_PURE_OFFSET;
        offsets.FROffset = FR_PURE_OFFSET;
        offsets.BLOffset = BL_PURE_OFFSET;
        offsets.BROffset = BR_PURE_OFFSET;

        return offsets;
    }

    public void saveToConfigFile() {
        Map<String, Double> swerveOffsetsMap = new HashMap<String, Double>();

        swerveOffsetsMap.put("FL_PURE_OFFSET", FLOffset.getDegrees());
        swerveOffsetsMap.put("FR_PURE_OFFSET", FROffset.getDegrees());
        swerveOffsetsMap.put("BL_PURE_OFFSET", BLOffset.getDegrees());
        swerveOffsetsMap.put("BR_PURE_OFFSET", BROffset.getDegrees());
    
        try {
            ObjectMapper objectMapper = new ObjectMapper();
            File swerveOffsetsFile = new File(OffsetConfigFile);

            System.out.println("Checking file at " + swerveOffsetsFile.getAbsolutePath());
            if (swerveOffsetsFile.exists()) {
                System.out.println("Deleting existing file");
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
}
