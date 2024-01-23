// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private CANSparkFlex shooterMotorL;
  private CANSparkFlex shooterMotorR;
  private CANSparkMax indexerMotorL;
  private CANSparkMax indexerMotorR;
  private PIDController shooterController;
  private PIDController indexerController;

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

    indexerMotorL = new CANSparkMax(ShooterConstants.CAN_INDEXER_L, MotorType.kBrushless);
    indexerMotorR = new CANSparkMax(ShooterConstants.CAN_INDEXER_R, MotorType.kBrushless);
    indexerMotorL.restoreFactoryDefaults();
    indexerMotorR.restoreFactoryDefaults();
    indexerMotorL.burnFlash();
    indexerMotorR.burnFlash();
    indexerMotorL.setSmartCurrentLimit(20);
    indexerMotorR.setSmartCurrentLimit(20);

    shooterController = new PIDController(ShooterConstants.CAN_SHOOTER_P, ShooterConstants.CAN_SHOOTER_I,
            ShooterConstants.CAN_SHOOTER_D);
    indexerController = new PIDController(ShooterConstants.CAN_INDEXER_P, ShooterConstants.CAN_INDEXER_I,
            ShooterConstants.CAN_INDEXER_D);
  }

  @Override
  public void periodic() {}

  public void driveShooter(double input) {
    shooterMotorL.set(input);
    SmartDashboard.putNumber("Shooter power", input);
    SmartDashboard.putNumber("Shooter RPM", shooterMotorL.getEncoder().getVelocity());
    shooterMotorR.set(-input);
  }

  public void rpmShooter(double setpoint, boolean isLeftMotor) {
    (isLeftMotor ? shooterMotorL : shooterMotorR).set(shooterController.calculate(
            (isLeftMotor ? shooterMotorL : shooterMotorR).getEncoder().getVelocity(), setpoint));
  }

  public void rpmIndexer(double setpoint, boolean isLeftIndexer) {
    (isLeftIndexer ? indexerMotorL : indexerMotorR).set(indexerController.calculate(
            (isLeftIndexer ? indexerMotorL : indexerMotorR).getEncoder().getVelocity(), setpoint));
  }

  public void rpmShooterBothSides(double setpoint) {
    rpmShooter(setpoint, true);
    rpmShooter(-setpoint, false);
  }

  public void rpmIndexerBothSides(double setpoint) {
    rpmIndexer(setpoint, true);
    rpmIndexer(-setpoint, false);
  }

  public int getShooterRpm() {
    return (int) (Math.abs(shooterMotorL.getEncoder().getVelocity()) +
            Math.abs(shooterMotorR.getEncoder().getVelocity())) / 2;
  }
}
