package frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeConstants {

    public static final int CAN_INTAKE = 21;
    public static final int CAN_INTAKE_PIVOT_L = 22;
    public static final int CAN_INTAKE_PIVOT_R = 23;

    public static final double INTAKE_Pivot_P = 0.012;
    public static final double INTAKE_Pivot_I = 0;
    public static final double INTAKE_Pivot_D = 0.025;

    // FF constants
    // See diagram its very usefull
    // deg -> rad = deg * .0175
    // inches -> meters = inches * .0254
    // lbs -> kg = lbs * .453592
    public static final double Alpha_Offset = -20*.0175;
    public static final double L3_WpivtoCm2 = 10*.0254;
    public static final double L2_WpivPerptoWpiv = 4.18*.0254;
    public static final double M2_Total_Mass_of_Intake = 15*.453592;
    public static final double INTAKE_Pivot_FF_Multiplier = 0.35;

}