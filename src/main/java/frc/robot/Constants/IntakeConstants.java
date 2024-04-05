package frc.robot.Constants;

public class IntakeConstants {

    public static final int CAN_INTAKE_1 = 21;
    public static final int CAN_INTAKE_2 = 24;
    public static final int CAN_INTAKE_PIVOT_L = 22;
    public static final int CAN_INTAKE_PIVOT_R = 23;

    public static final double INTAKE_Pivot_P = 0.013;//was .012 before 2/21 (evan changed)
    public static final double INTAKE_Pivot_I = 0;
    public static final double INTAKE_Pivot_D = 0.025;//was .025 before 2/21 (evan changed)

    public static final int INTAKE_CURRENT_LIMIT = 20;
    public static final int PIVOT_CURRENT_LIMIT = 40;

    public static final int PIVOT_ANGLE_MAX = 170;
    public static final int PIVOT_ANGLE_MIN = 0;

    // FF constants
    // See diagram its very usefull
    // deg -> rad = deg * .0175
    // inches -> meters = inches * .0254
    // lbs -> kg = lbs * .453592
    public static final double Alpha_Offset = -23.8*.0175;
    public static final double L3_WpivtoCm2 = 7.8*.0254;
    public static final double L2_WpivPerptoWpiv = 4.18*.0254;
    public static final double M2_Total_Mass_of_Intake = 14*.453592;
    public static final double INTAKE_Pivot_FF_Multiplier = 0.036; //.35 for basic ff

}