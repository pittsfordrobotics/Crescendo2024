// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import java.io.File;
import java.util.HashMap;
import java.util.Map;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
/** Add your docs here. */
public final class SwerveConstants {
    ObjectMapper objectMapper = new ObjectMapper();
    File swerveOffsetsFile = new File("/temp/SwerveOffsets.json");
    static Map<String, Double> swerveOffsetsMap = new HashMap<String, Double>();
    {
    try {
        swerveOffsetsMap = objectMapper.readValue(swerveOffsetsFile, new TypeReference<Map<String, Double>>(){});
    } catch (Exception e) {
        e.printStackTrace();
    }
    }
    
    
    public static final double driverControllerLeftDeadband = 0.1;
    public static final double driverControllerRightDeadband = 0.95;
    public static final int CAN_PIGEON = 0;

    public static int driveMaxRPM = 5676;

    // CAN ID of each motor
    public static final int CAN_BL_DRIVE = 5;
    public static final int CAN_BL_STEER = 6;
    public static final int CAN_BR_DRIVE = 1;
    public static final int CAN_BR_STEER = 2;
    public static final int CAN_FL_DRIVE = 7;
    public static final int CAN_FL_STEER = 8;
    public static final int CAN_FR_DRIVE = 3;
    public static final int CAN_FR_STEER = 4;
    /**
     *  Pinon    Gear Ratio    Max Speed [m/s] (approximate)
     *   12T 	   5.50:1 	      4.12
     *   13T 	   5.08:1 	      4.46
     *   14T 	   4.71:1         4.8
     */
    private enum MAX_SWERVE_GEARS {
        SLOW(12.0), MED(13.0), FAST(14.0);

        private final double gearRatio;
        private final double maxSpeed;
        MAX_SWERVE_GEARS(double pinion) {
            this.gearRatio = (45.0 * 22.0) / (pinion * 15.0);
            double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3);
            this.maxSpeed = ((driveMaxRPM / 60) * WHEEL_DIAMETER_METERS * Math.PI) / gearRatio;
        }
    }

    public static final MAX_SWERVE_GEARS GEAR_CONSTANTS = MAX_SWERVE_GEARS.FAST;
    public static final double MAX_LINEAR_VELOCITY_METERS_PER_SECOND = GEAR_CONSTANTS.maxSpeed; // 1678 ran 4.5 m/s in 2022
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 8.0; // from 1678

    public static final double DRIVE_GEAR_RATIO = GEAR_CONSTANTS.gearRatio;
    public static final double STEER_GEAR_RATIO = 46.2962962963;
//        CENTER OF WHEEL TO CENTER OF WHEEL
//        NOT CHASSIS LENGTH
    public static final double X_LENGTH_METERS = Units.inchesToMeters(24.5);
    public static final double Y_LENGTH_METERS = Units.inchesToMeters(24.5);
    public static final double BUMPER_WIDTH = Units.inchesToMeters(4);
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3);

    public static final Translation2d[] MODULE_OFFSETS = {
        new Translation2d(X_LENGTH_METERS / 2, Y_LENGTH_METERS / 2), // FL
        new Translation2d(X_LENGTH_METERS / 2, -Y_LENGTH_METERS / 2), // FR
        new Translation2d(-X_LENGTH_METERS / 2, Y_LENGTH_METERS / 2), // BL
        new Translation2d(-X_LENGTH_METERS / 2, -Y_LENGTH_METERS / 2), // BR
    };
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(MODULE_OFFSETS);

    // Measured module angles when using alignment tool
    public static final Rotation2d FL_PURE_OFFSET = Rotation2d.fromDegrees(swerveOffsetsMap.getOrDefault("FL_PURE_OFFSET", 0.0));
    public static final Rotation2d FR_PURE_OFFSET = Rotation2d.fromDegrees(swerveOffsetsMap.getOrDefault("FR_PURE_OFFSET", 0.0));
    public static final Rotation2d BL_PURE_OFFSET = Rotation2d.fromDegrees(swerveOffsetsMap.getOrDefault("BL_PURE_OFFSET", 0.0));
    public static final Rotation2d BR_PURE_OFFSET = Rotation2d.fromDegrees(swerveOffsetsMap.getOrDefault("BR_PURE_OFFSET", 0.0));

    // Add angles of offset based on mounting angle of modules
    public static final Rotation2d FL_OFFSET = FL_PURE_OFFSET.plus(Rotation2d.fromDegrees(-90));
    public static final Rotation2d FR_OFFSET = FR_PURE_OFFSET.plus(Rotation2d.fromDegrees(0));
    public static final Rotation2d BL_OFFSET = BL_PURE_OFFSET.plus(Rotation2d.fromDegrees(-180));
    public static final Rotation2d BR_OFFSET = BR_PURE_OFFSET.plus(Rotation2d.fromDegrees(-270));
    
    // controlling module wheel speed
    public static final double MODULE_DRIVE_P = 0.1;
    public static final double MODULE_DRIVE_I = 0;
    public static final double MODULE_DRIVE_D = 0;

    // // feedforward for module from SysID
    // public static final double MODULE_DRIVE_S = 0;
    // public static final double MODULE_DRIVE_V = 0;
    // public static final double MODULE_DRIVE_A = 0;
    public static final double MODULE_DRIVE_FF = 1 / MAX_LINEAR_VELOCITY_METERS_PER_SECOND; // feedforward for drive motors

    // controlling module angle
    public static final double MODULE_STEER_P = 2.5;
    public static final double MODULE_STEER_I = 0;
    public static final double MODULE_STEER_D = 0;
    //public static final double MODULE_STEER_FF_OL = Robot.isReal() ? 0.6 : 0.27;
    //public static final double MODULE_STEER_FF_CL = Robot.isReal() ? 0.8 : 0.33;
    //public static final double AUTO_ROTATE_P = 5;
    //public static final double AUTO_ROTATE_I = 0;
    //public static final double AUTO_ROTATE_D = 0;
    //public static final double AUTO_ROTATE_TOLERANCE = 0.05;

    public static final double MODULE_ROTATION_RATE_LIMIT = 10; // max allowed rotation rate of swerve in radians per second
    
    // New PID values for robot angle
    public static final double ROBOT_ANG_P = 10;
    public static final double ROBOT_ANG_I = 0;
    public static final double ROBOT_ANG_D = 0;

    // Determines how the swerve turns (True = Charged Up way, False = New(and better) way)
    public static final boolean TurnWithRightStickX = false;

}
