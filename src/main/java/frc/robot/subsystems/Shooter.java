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
  private CANSparkMax shooterpivot_L;
  private CANSparkMax shooterpivot_R;
  private PIDController shooterController;
  private PIDController indexerController;
  private PIDController ShooterPivotController;
;


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

    shooterpivot_L = new CANSparkMax(ShooterConstants.CAN_SHOOTER_PIVOT_L, MotorType.kBrushless);
    shooterpivot_R = new CANSparkMax(ShooterConstants.CAN_SHOOTER_PIVOT_R, MotorType.kBrushless);
    shooterpivot_L.restoreFactoryDefaults();
    shooterpivot_R.restoreFactoryDefaults();
    shooterpivot_L.burnFlash();
    shooterpivot_R.burnFlash();
    shooterpivot_L.setSmartCurrentLimit(20);
    shooterpivot_R.setSmartCurrentLimit(20);
    shooterpivot_L.follow(shooterpivot_R, true);

    shooterController = new PIDController(ShooterConstants.CAN_SHOOTER_P, ShooterConstants.CAN_SHOOTER_I,
            ShooterConstants.CAN_SHOOTER_D);
    indexerController = new PIDController(ShooterConstants.CAN_INDEXER_P, ShooterConstants.CAN_INDEXER_I,
            ShooterConstants.CAN_INDEXER_D);
    ShooterPivotController = new PIDController(ShooterConstants.CAN_SHOOTER_Pivot_P, ShooterConstants.CAN_SHOOTER_Pivot_I,
            ShooterConstants.CAN_SHOOTER_Pivot_D);
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

  public void setpivotangle(double setpoint){
  shooterpivot_R.set(ShooterPivotController.calculate(
    shooterpivot_R.getEncoder().getPosition(), setpoint));
  }

  


}
