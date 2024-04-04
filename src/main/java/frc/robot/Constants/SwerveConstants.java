package frc.robot.Constants;

import frc.robot.lib.SwerveModuleConstants;

public class SwerveConstants {

    private static SwerveModuleConstants FRONT_LEFT_CONSTANTS = new SwerveModuleConstants(0.3199, 2.6512, 10.006);
    private static SwerveModuleConstants FRONT_RIGHT_CONSTANTS = new SwerveModuleConstants(0.23351, 2.6361, 13.654);
    private static SwerveModuleConstants BACK_LEFT_CONSTANTS = new SwerveModuleConstants(0.26965, 2.6011, 13.234);
    private static SwerveModuleConstants BACK_RIGHT_CONSTANTS = new SwerveModuleConstants(0.25825, 2.7213, 12.562);
    public static final SwerveModuleConstants[] MODULE_CONSTANTS = {FRONT_LEFT_CONSTANTS, FRONT_RIGHT_CONSTANTS, BACK_LEFT_CONSTANTS, BACK_RIGHT_CONSTANTS};

    // public class BACK_LEFT {
    //     public static final double drive_kS = 0.26965;
    //     public static final double drive_kV = 2.6011;
    //     public static final double drive_kA = 13.234;
    // }
    // public class BACK_RIGHT {
    //     public static final double drive_kS = 0.25825;
    //     public static final double drive_kV = 2.7213;
    //     public static final double drive_kA = 12.562;
    // }
    // public class FRONT_LEFT {
    //     public static final double drive_kS = 0.3199;
    //     public static final double drive_kV = 2.6512;
    //     public static final double drive_kA = 10.006;

    // }
    // public class FRONT_RIGHT {
    //     public static final double drive_kS = 0.23351;
    //     public static final double drive_kV = 2.6361;
    //     public static final double drive_kA = 13.654;
    // }
}
