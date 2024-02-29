// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

/** Add your docs here. */
public class RobotConstants {
    // Positive RPM pushes out the note
    // Positive angle movement moves the shooter or intake up

    // Indexer speeds
    public static final double INDEXER_SHOOT_SPEED = 1;
    public static final double INDEXER_IDLE_SPEED = -.1;

    // Intake "state"
    public static final double INTAKE_IntakePivotAngle = 0;
    public static final double INTAKE_ShooterPivotAngle = 0;
    public static final double INTAKE_IntakeSpeed = -1;
    public static final double INTAKE_ShooterRPM = -3000;

    // Stored "state"
    public static final double STORED_ShooterPivotAngle = 0;
    public static final double STORED_IntakePivotAngle = 170;
    public static final double STORED_ShooterRPM = 2000;
    public static final double STORED_IntakeSpeed = 0;

    // Amp "state"
    public static final double AMP_ShooterPivotAngle = 48;
    public static final double AMP_IntakePivotAngle = 0;
    public static final double AMP_ShooterRPM = 2000;
    public static final double AMP_IntakeSpeed = .6;

    // Speaker_SUBWOOF "state"
    public static final double SUBWOOF_ShooterPivotAngle = 53;
    public static final double SUBWOOF_IntakePivotAngle = 35;
    public static final double SUBWOOF_ShooterRPM = 5400;
    public static final double SUBWOOF_IntakeSpeed = 0;
    
    // Speaker_PODIUM "state"
    public static final double PODIUM_ShooterPivotAngle = 40.25;
    public static final double PODIUM_IntakePivotAngle = 35;
    public static final double PODIUM_ShooterRPM = 6000;
    public static final double PODIUM_IntakeSpeed = 0;

    // new AMP COMMAND "STATE"
    public static final double NEWAMP_ShooterPivotAngle_STAGE1 = 50;
    public static final double NEWAMP_IntakePivotAngle_STAGE1 = 0;
    public static final double NEWAMP_ShooterRPM_STAGE1 = 2000;
    public static final double NEWAMP_IntakeSpeed_STAGE1 = .7;
    public static final double NEWAMP_WaitTime_STAGE1 = .1875;

    public static final double NEWAMP_ShooterPivotAngle_STAGE2 = 20;
    public static final double NEWAMP_IntakePivotAngle_STAGE2 = 160;
    public static final double NEWAMP_ShooterRPM_STAGE2 = 0;
    public static final double NEWAMP_IntakeSpeed_STAGE2 = 0;

    public static final double NEWAMP_IntakeSpeed_ShootOut = .7;

}
