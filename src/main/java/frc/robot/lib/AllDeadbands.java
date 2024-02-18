// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

/** Contains multiple different shaped deadbands for inputs, math, etc. */
public class AllDeadbands {
    /** Applies a circular deadband to a double array
     * @param inputs the array of inputs to apply deadband to
     * @param deadband the radius of the circular deadband
     * @return the array of inputs, zero if within deadband, otherwise unchanged
     */
    public static double[] applyCircularDeadband(double[] inputs, double deadband) {
        double radiusSquared = 0;
        int len = 0;
        double[] outputs;
        for (double input : inputs) {
            radiusSquared += Math.pow(input, 2);
            len ++;
        }
        double radius = Math.sqrt(radiusSquared);
        if(radius < deadband) {
            outputs = new double[len];
        } else {
            outputs = inputs;
        }
        return outputs;
    }
    /** Applies a circular deadband to a double array and scales output magnitude from 0 to 1 by distance from inside of deadband.
     * @param inputs the array of inputs to apply deadband to
     * @param deadband the radius of the circular deadband
     * @return the array of inputs, zero if within deadband, otherwise unchanged
     */
    public static double[] applyScalingCircularDeadband(double[] inputs, double deadband) {
        double radiusSquared = 0;
        double scaledRadius = 0;
        int len = 0;
        double[] outputs;
        for (double input : inputs) {
            radiusSquared += Math.pow(input, 2);
            len ++;
        }
        double radius = Math.sqrt(radiusSquared);
        outputs = new double[len];
        if(radius >= deadband) {
            scaledRadius = radius - deadband;
            for(int i = 0; i < len; i++) {
                outputs[i] = inputs[i] * scaledRadius / radius;
            }
        }
        return outputs;
    }
}
