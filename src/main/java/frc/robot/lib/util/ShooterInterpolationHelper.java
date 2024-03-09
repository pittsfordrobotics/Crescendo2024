// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.SwerveSubsystem;


public class ShooterInterpolationHelper {

    /** Takes a distance from the cetner of the robot to the subwoof apriltag (meters)
    * and returns an angle to set the shooter to
    */
    public static double getShooterAngle(double distance) {
        if(Units.metersToInches(distance) < 87.0) {
            return RobotConstants.SUBWOOF_ShooterPivotAngle; // TODO: change to distanceanglemap when the subwoofangle and the subwoofangle in the distance map are the same
        }
        if(Units.metersToInches(distance) > 154.0) {
            return ShooterConstants.DISTANCE_ANGLE_MAP.get(Units.inchesToMeters(154));
        }
        return ShooterConstants.DISTANCE_ANGLE_MAP.get(distance);
    }

    /** Takes a distance from the cetner of the robot to the subwoof apriltag (meters)
    * and returns an RPM to set the shooter to
    */
    public static double getShooterRPM(double distance) {
        // decreases RPM from usual 6000 if close to the speaker
        return 5400;
    }


}