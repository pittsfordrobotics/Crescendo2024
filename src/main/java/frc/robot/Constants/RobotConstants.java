// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

/** Add your docs here. */
public class RobotConstants {
    // Positive RPM pushes out the note
    // Positive angle movement moves the shooter or intake up

    // Indexer speeds
    public static final double INDEXER_SHOOT_SPEED = .5;
    public static final double INDEXER_IDLE_SPEED = -.1;

    // Intake "state"
    public static final double INTAKE_IntakePivotAngle = 0;
    public static final double INTAKE_ShooterPivotAngle = 0;
    public static final double INTAKE_Intake_Speed = -.5;
    public static final double INTAKE_Shooter_Speed = -1250;

    // Stored "state"
    public static final double STORED_ShooterPivotAngle = 0;
    public static final double STORED_IntakePivotAngle = 170;
    public static final double STORED_ShooterRPM = 2000;
    public static final double STORED_IntakeSpeed = 0;

    // Amp "state"
    public static final double AMP_ShooterPivotAngle = 60;
    public static final double AMP_IntakePivotAngle = 0;
    public static final double AMP_ShooterRPM = 2000;
    public static final double AMP_IntakeSpeed = 1;

    // Speaker_SUBWOOF "state"
    public static final double SUBWOOF_ShooterPivotAngle = 53;
    public static final double SUBWOOF_IntakePivotAngle = 70;
    public static final double SUBWOOF_ShooterRPM = 5400;
    public static final double SUBWOOF_IntakeSpeed = 0;
    
    // Speaker_PODIUM "state"
    public static final double PODIUM_ShooterPivotAngle = 40;
    public static final double PODIUM_IntakePivotAngle = 70;
    public static final double PODIUM_ShooterRPM = 6500;
    public static final double PODIUM_IntakeSpeed = 0;
}
