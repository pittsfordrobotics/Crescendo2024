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
    public Rotation2d[] getSwerveOffsets() {
        Map<String, Double> swerveOffsetsMap = new HashMap<String, Double>();
        ObjectMapper objectMapper = new ObjectMapper();
        File swerveOffsetsFile = new File("/home/lvuser/SwerveOffsets.json");
        if(!swerveOffsetsFile.exists()) {
            try {
                System.out.println("Creating new file");
                swerveOffsetsFile.createNewFile();
            } catch (IOException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
        try {
            System.out.println("Reading file");
            swerveOffsetsMap = objectMapper.readValue(swerveOffsetsFile, new TypeReference<Map<String, Double>>(){});
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

        // Add angles of offset based on mounting angle of modules
        Rotation2d FL_OFFSET = FL_PURE_OFFSET.plus(Rotation2d.fromDegrees(-90));
        Rotation2d FR_OFFSET = FR_PURE_OFFSET.plus(Rotation2d.fromDegrees(0));
        Rotation2d BL_OFFSET = BL_PURE_OFFSET.plus(Rotation2d.fromDegrees(-180));
        Rotation2d BR_OFFSET = BR_PURE_OFFSET.plus(Rotation2d.fromDegrees(-270));

        return new Rotation2d[]{FL_PURE_OFFSET, FR_PURE_OFFSET, BL_PURE_OFFSET, BR_PURE_OFFSET};
    }
    }
