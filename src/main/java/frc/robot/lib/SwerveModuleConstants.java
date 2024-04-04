// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

/** Add your docs here. */
public class SwerveModuleConstants {
    public final double drive_kS;
    public final double drive_kV;
    public final double drive_kA;
    public SwerveModuleConstants(double drive_kS, double drive_kV, double drive_kA) {
        this.drive_kS = drive_kS;
        this.drive_kV = drive_kV;
        this.drive_kA = drive_kA;
    }
}
