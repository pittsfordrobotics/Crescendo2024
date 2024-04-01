// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
        Shuffleboard.getTab("FFcalc").addDouble("ShoulderFF", this::calculateShooterFF);
        Shuffleboard.getTab("FFcalc").addDouble("IntakeFF", this::calculateIntakeFF);        
        SmartDashboard.putNumber("Shooter FF Mult", ShooterConstants.SHOOTER_Pivot_FF_Multiplier);
        SmartDashboard.putNumber("Intake FF Mult", IntakeConstants.INTAKE_Pivot_FF_Multiplier);
    }

    public void updateShooterAngle(DoubleSupplier shooterAngle) {
        this.shooterAngle = shooterAngle;
    }

    public void updateIntakePivotAngle(DoubleSupplier intakePivotAngle) {
        this.intakePivotAngle = intakePivotAngle;
    }

    public double calculateShooterFF() {

        double ShooterMultiplier = ShooterConstants.SHOOTER_Pivot_FF_Multiplier;
        // double ShooterMultiplier = SmartDashboard.getNumber("Shooter FF Mult",
        //         ShooterConstants.SHOOTER_Pivot_FF_Multiplier);

        Rotation2d theta = Rotation2d.fromDegrees(this.shooterAngle.getAsDouble());
        Rotation2d alpha = Rotation2d.fromDegrees(this.intakePivotAngle.getAsDouble());

        double L1 = ShooterConstants.L1_SpivtoWpivperp;
        double L1CM = ShooterConstants.L1CM1_SpivtoCM1;
        double L2 = IntakeConstants.L2_WpivPerptoWpiv;
        double L3 = IntakeConstants.L3_WpivtoCm2;
        double M1 = ShooterConstants.M1_Total_Mass_of_Shooter;
        double M2 = IntakeConstants.M2_Total_Mass_of_Intake;
        double MT = M1 + M2;
        Rotation2d Theta_CM = theta.plus(Rotation2d.fromDegrees(ShooterConstants.Theta_Offset));
        Rotation2d Alpha_CM = alpha.plus(Rotation2d.fromDegrees(IntakeConstants.Alpha_Offset));

        double CM2X = (L1 * Math.cos(theta.getRadians()))
                + (L2 * Math.cos(theta.plus(Rotation2d.fromDegrees(90)).getRadians())
                        + (L3 * Math.cos(theta.plus(Alpha_CM).getRadians())));

        double CM1X = L1CM * Math.cos(Theta_CM.getRadians());

        double CMX = (CM2X * M2 + CM1X * M1) / MT;

        double TotalTorque_ShoulderPiv = CMX * 9.8 * MT;

        if (theta.getDegrees() < 5) {
            return 0;
        }


        return TotalTorque_ShoulderPiv * ShooterMultiplier;

        // Simple Multiplier
        // return ShooterMultiplier * Math.cos(theta.getRadians());
    }

    public double calculateIntakeFF() {

        double IntakeMultiplier = IntakeConstants.INTAKE_Pivot_FF_Multiplier;
        // double IntakeMultiplier = SmartDashboard.getNumber("Intake FF Mult",
        //         IntakeConstants.INTAKE_Pivot_FF_Multiplier);

        Rotation2d theta = Rotation2d.fromDegrees(this.shooterAngle.getAsDouble());
        Rotation2d alpha = Rotation2d.fromDegrees(this.intakePivotAngle.getAsDouble());

        double L3 = IntakeConstants.L3_WpivtoCm2;
        double M2 = IntakeConstants.M2_Total_Mass_of_Intake;
        Rotation2d Alpha_CM = alpha.plus(Rotation2d.fromDegrees(IntakeConstants.Alpha_Offset));

        double TotalTorque_IntakePiv = (Math.cos(theta.plus(Alpha_CM).getRadians()) * 9.8 * (M2)) * L3;

        if(alpha.getDegrees() < 15){
            return 0;
        }

        return TotalTorque_IntakePiv * IntakeMultiplier;
        // return IntakeMultiplier * Math.cos(theta.plus(Alpha_CM).getRadians());
    }
}
