// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

/** Shooter Constants */
public final class ShooterConstants {
    public static final int CAN_SHOOTER_L = 11;
    public static final int CAN_SHOOTER_R = 12;

    public static final int CAN_INDEXER_L = 13;
    public static final int CAN_INDEXER_R = 14;

    public static final int CAN_SHOOTER_PIVOT_L = 15;
    public static final int CAN_SHOOTER_PIVOT_R = 16;

    public static final double SHOOTER_P = 0.0004;
    public static final double SHOOTER_I = 0;
    public static final double SHOOTER_D = 0;

    public static final double SHOOTER_Pivot_P = 0.025;// was .03 before 2/21 (evan changed)
    public static final double SHOOTER_Pivot_I = 0;
    public static final double SHOOTER_Pivot_D = 0.35;// was .3 before 2/21 (evan changed)

    public static final double SHOOTER_L_FFGain = 0.000155;
    public static final double SHOOTER_R_FFGain = 0.000158;


    // FF constants
    // See diagram its very usefull
    // deg -> rad = deg * .0175
    // inches -> meters = inches * .0254
    // lbs -> kg = lbs * .453592
    public static final double Theta_Offset = -2 * 0.0175;
    public static final double L1_SpivtoWpivperp = 14.631 * 0.0254;
    public static final double L1CM1_SpivtoCM1 = 7.387 * 0.0254;
    public static final double M1_Total_Mass_of_Shooter = 23 * 0.453592;
    public static final double SHOOTER_Pivot_FF_Multiplier = 0.014;//.83 for the basic ff

    // key: distance from center of robot to middle pose of subwoof projected down (inches); value: shooter angle (degrees)
    public static final InterpolatingDoubleTreeMap DISTANCE_ANGLE_MAP = new InterpolatingDoubleTreeMap();
    static {
        DISTANCE_ANGLE_MAP.put(Units.inchesToMeters(53.6), 56.0); // Speaker
        DISTANCE_ANGLE_MAP.put(Units.inchesToMeters(87.0), 45.0); // Middle auto second shoot
        DISTANCE_ANGLE_MAP.put(Units.inchesToMeters(104.0), 41.5); // Podium auto second shoot
        DISTANCE_ANGLE_MAP.put(Units.inchesToMeters(115.0), 40.25); // Podium
        DISTANCE_ANGLE_MAP.put(3.26497, 39.0); // Amp auto second shoot
        DISTANCE_ANGLE_MAP.put(Units.inchesToMeters(154.0), 37.8); // Tested distance
    }
}
