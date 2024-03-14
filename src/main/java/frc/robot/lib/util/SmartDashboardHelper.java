// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.util;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class SmartDashboardHelper {

    public static DoubleSupplier truncatedDoubleSupplier(DoubleSupplier supplier, int decimalPlaces) {
        return () -> {
            double value = supplier.getAsDouble();
            double multiplier = Math.pow(10, decimalPlaces);
            return Math.floor(value * multiplier) / multiplier;
        };
    }
}
