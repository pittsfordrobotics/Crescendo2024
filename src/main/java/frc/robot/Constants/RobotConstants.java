// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

/** Add your docs here. */
public class RobotConstants {
    public static final double SHOOTER_PIVOT_ANGLE_TOLERANCE = 2;
    public static final double INTAKE_PIVOT_ANGLE_TOLERANCE = 2;
    public static final double SHOOTER_RPM_TOLERANCE = 100;
    public static final double INDEXER_SHOOT_SPEED = .5;
    public static final double INDEXER_HOLD_SPEED = -.1;

    // Intake "state"
    public static final double INTAKE_IntakePivotAngle = 0;
    public static final double INTAKE_ShooterPivotAngle = 180;
    public static final double INTAKE_Intake_Speed = -.5;
    public static final double INTAKE_Indexer_Speed = -.5;
    public static final double INTAKE_Shooter_Speed = -1000;

    // Shooter "state"
    public static final double STORED_ShooterPivotAngle = 0;
    public static final double STORED_IntakePivotAngle = 180;
    public static final double STORED_ShooterRPM = 0;
    public static final double STORED_IntakeSpeed = 0;

    // Amp "state"
    public static final double AMP_ShooterPivotAngle = 60;
    public static final double AMP_IntakePivotAngle = 0;
    public static final double AMP_ShooterRPM = 1000;
    public static final double AMP_IntakeSpeed = 1;

    // Speaker "state"
    public static final double SPEAKER_ShooterPivotAngle = 55;
    public static final double SPEAKER_IntakePivotAngle = 90;
    public static final double SPEAKER_ShooterRPM = 5000;
    public static final double SPEAKER_IntakeSpeed = 0;

}
