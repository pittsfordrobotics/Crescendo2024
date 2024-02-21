// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

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

    public static final double SHOOTER_Pivot_P = 0.03;//.03
    public static final double SHOOTER_Pivot_I = 0;
    public static final double SHOOTER_Pivot_D = 0.3;//.3

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
    public static final double SHOOTER_Pivot_FF_Multiplier = 0.017;//.83 for the basic ff
}
