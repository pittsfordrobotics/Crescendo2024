// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

/** Add your docs here. */
public class FFCalculator {

    private static FFCalculator instance;

    public static FFCalculator getInstance() {
        if (instance == null) {
            instance = new FFCalculator();
        }
        return instance;
    }

    private DoubleSupplier shooterAngle = () -> 0.0;
    private DoubleSupplier intakePivotAngle = () -> 0.0;

    private FFCalculator() {
    }

    public void updateShooterAngle(DoubleSupplier shooterAngle) {
        this.shooterAngle = shooterAngle;
    }

    public void updateIntakePivotAngle(DoubleSupplier intakePivotAngle) {
        this.intakePivotAngle = intakePivotAngle;
    }

    public double calculateShooterFF() {

        // double ShooterMultiplier = ShooterConstants.SHOOTER_Pivot_FF_Multiplier;
        SmartDashboard.putNumber("Shooter FF Mult", ShooterConstants.SHOOTER_Pivot_FF_Multiplier);
        double ShooterMultiplier = SmartDashboard.getNumber("Shooter FF Mult", ShooterConstants.SHOOTER_Pivot_FF_Multiplier);
        
        double theta = this.shooterAngle.getAsDouble();
        double alpha = this.intakePivotAngle.getAsDouble();

        double L1 = ShooterConstants.L1_SpivtoWpivperp;
        double L1CM = ShooterConstants.L1CM1_SpivtoCM1;
        double L2 = IntakeConstants.L2_WpivPerptoWpiv;
        double L3 = IntakeConstants.L3_WpivtoCm2;
        double M1 = ShooterConstants.M1_Total_Mass_of_Shooter;
        double M2 = IntakeConstants.M2_Total_Mass_of_Intake;
        double Theta_CM = theta + ShooterConstants.Theta_Offset;
        double Alpha_CM = alpha + IntakeConstants.Alpha_Offset;

        double CM2X = (L1 * Math.cos(theta)) + (L2 * Math.cos(90 + theta)) + (L3 * Math.cos(Alpha_CM + theta));
        double CM2Y = (L1 * Math.sin(theta)) + (L2 * Math.sin(90 + theta)) + (L3 * Math.sin(Alpha_CM + theta));

        double CM1X = L1CM * Math.cos(Theta_CM);
        double CM1Y = L1CM * Math.sin(Theta_CM);

        double CMX = (CM2X * M2 + CM1X * M1) / (M1 + M2);
        double CMY = (CM2Y * M2 + CM1Y * M1) / (M1 + M2);

        // Change to be simplified like get rid of the atan crap and NO DIVIDING
        double TotalTorque_ShoulderPiv = (Math.cos(Math.atan(CMY / CMX)) * 9.8 * (M1 + M2))
                * Math.sqrt((CMX * CMX) + (CMY * CMY));
        // return TotalTorque_ShoulderPiv * ShooterMultiplier;
        return ShooterMultiplier * Math.cos(theta);
    }

    public double calculateIntakeFF() {

        // double IntakeMultiplier = IntakeConstants.INTAKE_Pivot_FF_Multiplier;
        SmartDashboard.putNumber("Intake FF Mult", IntakeConstants.INTAKE_Pivot_FF_Multiplier);
        double IntakeMultiplier = SmartDashboard.getNumber("Intake FF Mult", IntakeConstants.INTAKE_Pivot_FF_Multiplier);

        double theta = shooterAngle.getAsDouble();
        double alpha = intakePivotAngle.getAsDouble();

        double L3 = IntakeConstants.L3_WpivtoCm2;
        double M2 = IntakeConstants.M2_Total_Mass_of_Intake;
        double Alpha_CM = alpha + IntakeConstants.Alpha_Offset;

        double TotalTorque_IntakePiv = (Math.cos(theta + Alpha_CM) * 9.8 * (M2)) * L3;
        // return TotalTorque_IntakePiv * IntakeMultiplier;        
        return IntakeMultiplier * Math.cos(theta + Alpha_CM);
    }
}
