// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;


public class ShooterInterpolationHelper {

    /** Takes a distance from the cetner of the robot to the subwoof apriltag (meters)
    * and returns an angle to set the shooter to
    */
    public static double getShooterAngle(double distance) {
        if(Units.metersToInches(distance) < 56) {
            return RobotConstants.SUBWOOF_ShooterPivotAngle; // TODO: change to distanceanglemap when the subwoofangle and the subwoofangle in the distance map are the same
        }
        if(Units.metersToInches(distance) > 130.0) {
            return ShooterConstants.DISTANCE_ANGLE_MAP.get(Units.inchesToMeters(130));
        }
        return ShooterConstants.DISTANCE_ANGLE_MAP.get(distance);
    }
    public static DoubleSupplier getShooterAngle (DoubleSupplier distance){
        // try{ 
        //     System.out.println("Perceived distance to speaker opening:" + distance.getAsDouble());
        // } catch (Exception e) {
        // }
        return (() -> getShooterAngle(distance.getAsDouble()));
    }

    /** Takes a distance from the cetner of the robot to the subwoof apriltag (meters)
    * and returns an RPM to set the shooter to
    */
    public static double getShooterRPM(double distance) {
        // decreases RPM from usual 6000 if close to the speaker
        return 5400;
    }

    public static DoubleSupplier getShooterRPM (DoubleSupplier distance) {
        return ()-> getShooterRPM(distance.getAsDouble());
    }


}
