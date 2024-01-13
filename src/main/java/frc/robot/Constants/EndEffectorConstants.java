// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

/** Add your docs here. */
public final class EndEffectorConstants {
    public static CANSparkMax leftMotor = new CANSparkMax(13, MotorType.kBrushless);
    public static CANSparkMax rightMotor = new CANSparkMax(9, MotorType.kBrushless);
}
