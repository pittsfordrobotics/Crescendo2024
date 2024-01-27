// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import frc.robot.subsystems.Vision.VisionIO;
import frc.robot.subsystems.Vision.VisionIOLimelight;

/** Add your docs here. */
public final class VisionConstants {

    // the Limelight Name is the name of the network table i think this is what is should be
    public final static VisionIO LIMELIGHT1;
    public final static String LIMELIGHT1_NAME = "limelight-one";
    
    public final static VisionIO LIMELIGHT2;
    public final static String LIMELIGHT2_NAME = "limelight-two";
    static {
        LIMELIGHT1 = new VisionIOLimelight(LIMELIGHT1_NAME);
        LIMELIGHT2 = new VisionIOLimelight(LIMELIGHT2_NAME);
    }

    public static final double FIELD_BORDER_MARGIN = 0.5;
    public static final double Z_MARGIN = 0.75;
    public static final double XY_STD_DEV_COEF = 0.01;
    public static final double THETA_STD_DEV_COEF = 0.01;
    public static final double TARGET_LOG_SECONDS = 0.1;
}
