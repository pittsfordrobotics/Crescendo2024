// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  CANSparkFlex shooterMotorL;
  CANSparkFlex shooterMotorR;

  /** Creates a new Shooter. */
  public Shooter() {
    shooterMotorL = new CANSparkFlex(ShooterConstants.CAN_SHOOTER_L, MotorType.kBrushless);
    shooterMotorL.restoreFactoryDefaults();
    // shooterMotorL.setSmartCurrentLimit(40);
    shooterMotorL.burnFlash();
    shooterMotorR = new CANSparkFlex(ShooterConstants.CAN_SHOOTER_R, MotorType.kBrushless);
    shooterMotorR.restoreFactoryDefaults();
    // shooterMotorR.setSmartCurrentLimit(40);
    shooterMotorR.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveShooter(double input) {
    shooterMotorL.set(input);
    SmartDashboard.putNumber("Shooter power", input);
    SmartDashboard.putNumber("Shooter RPM", shooterMotorL.getEncoder().getVelocity());
    shooterMotorR.set(-input);
  }
}
