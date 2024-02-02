// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DisabledInstantCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private CANSparkFlex shooterMotorL;
  private CANSparkFlex shooterMotorR;
  private CANSparkMax indexerMotorL;
  private CANSparkMax indexerMotorR;
  private CANSparkMax shooterpivot_L;
  private CANSparkMax shooterpivot_R;
  private SparkLimitSwitch backLimitSwitch;

  private SparkPIDController shooterpivotRPID;
  private SparkPIDController shooterpivotLPID;
  private SparkPIDController RMPShooterLPid;
  private SparkPIDController RMPShooterRPid;

  private SparkAbsoluteEncoder shooterpivot_R_ABSEncoder;
  private SparkAbsoluteEncoder shooterpivot_L_ABSEncoder;

  // make into a singleton
  private final static Shooter instance = new Shooter();
  public static Shooter getInstance() {
    return instance;
  }

  /** Creates a new Shooter. */
  public Shooter() {
    // Shooter Motor L
    shooterMotorL = new CANSparkFlex(ShooterConstants.CAN_SHOOTER_L, MotorType.kBrushless);
    shooterMotorL.restoreFactoryDefaults();
    // shooterMotorL.setSmartCurrentLimit(40);
    shooterMotorL.setInverted(true);
    shooterMotorL.burnFlash();
    // Shooter Motor R
    shooterMotorR = new CANSparkFlex(ShooterConstants.CAN_SHOOTER_R, MotorType.kBrushless);
    shooterMotorR.restoreFactoryDefaults();
    // shooterMotorR.setSmartCurrentLimit(40);
    shooterMotorR.burnFlash();

    // Index Motor R (Leader)
    indexerMotorR = new CANSparkMax(ShooterConstants.CAN_INDEXER_R, MotorType.kBrushless);
    indexerMotorR.restoreFactoryDefaults();
    indexerMotorR.setSmartCurrentLimit(20);    
    backLimitSwitch = indexerMotorR.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    indexerMotorR.burnFlash();
    // Index Motor L
    indexerMotorL = new CANSparkMax(ShooterConstants.CAN_INDEXER_L, MotorType.kBrushless);
    indexerMotorL.restoreFactoryDefaults();
    indexerMotorL.setSmartCurrentLimit(20);
    indexerMotorL.follow(indexerMotorR, true);
    indexerMotorL.burnFlash();

    // ShooterPivot R (Leader)
    shooterpivot_R = new CANSparkMax(ShooterConstants.CAN_SHOOTER_PIVOT_R, MotorType.kBrushless);
    shooterpivot_R.restoreFactoryDefaults();
    shooterpivot_R.setSmartCurrentLimit(20);
    shooterpivot_R_ABSEncoder = shooterpivot_R.getAbsoluteEncoder(Type.kDutyCycle);    
    shooterpivot_R.burnFlash();
    // ShooterPivot L
    shooterpivot_L = new CANSparkMax(ShooterConstants.CAN_SHOOTER_PIVOT_L, MotorType.kBrushless);
    shooterpivot_L.restoreFactoryDefaults();
    shooterpivot_L.setSmartCurrentLimit(20);
    shooterpivot_L.follow(shooterpivot_R, true);
    shooterpivot_L_ABSEncoder = shooterpivot_L.getAbsoluteEncoder(Type.kDutyCycle);
    shooterpivot_L.burnFlash();

    // Limit Switch

    // PID Controllers

    // ShooterPivotPID R
    shooterpivotRPID = shooterpivot_R.getPIDController();
    shooterpivotRPID.setFeedbackDevice(shooterpivot_R_ABSEncoder);
    shooterpivotRPID.setP(ShooterConstants.SHOOTER_Pivot_P);
    shooterpivotRPID.setI(ShooterConstants.SHOOTER_Pivot_I);
    shooterpivotRPID.setD(ShooterConstants.SHOOTER_Pivot_D);
    // // ShooterPivotPID L
    // shooterpivotLPID = shooterpivot_L.getPIDController();
    // shooterpivotLPID.setFeedbackDevice(shooterpivot_L_ABSEncoder);
    // shooterpivotLPID.setP(ShooterConstants.SHOOTER_Pivot_P);
    // shooterpivotLPID.setI(ShooterConstants.SHOOTER_Pivot_I);
    // shooterpivotLPID.setD(ShooterConstants.SHOOTER_Pivot_D);

    // RPMShooterPID_L
    RMPShooterLPid = shooterpivot_L.getPIDController();
    RMPShooterLPid.setFeedbackDevice(shooterMotorL.getEncoder());
    RMPShooterLPid.setP(ShooterConstants.SHOOTER_P);
    RMPShooterLPid.setI(ShooterConstants.SHOOTER_I);
    RMPShooterLPid.setD(ShooterConstants.SHOOTER_D);
    // RPMShooterPID_R
    RMPShooterRPid = shooterpivot_R.getPIDController();
    RMPShooterRPid.setFeedbackDevice(shooterMotorR.getEncoder());
    RMPShooterRPid.setP(ShooterConstants.SHOOTER_P);
    RMPShooterRPid.setI(ShooterConstants.SHOOTER_I);
    RMPShooterRPid.setD(ShooterConstants.SHOOTER_D);

    Shuffleboard.getTab("SHOOTER").add("Shooter Pivot Set Coast", new DisabledInstantCommand(this::coastShooter));
    Shuffleboard.getTab("SHOOTER").add("Shooter Pivot Set Brake", new DisabledInstantCommand(this::brakeShooter));
    Shuffleboard.getTab("SHOOTER").add("Shooter Pivot Zero", new DisabledInstantCommand(this::zeroPivot));
  }

  @Override
  public void periodic() {
    Shuffleboard.getTab("SHOOTER").add("Shooter RPM", this.getShooterRpm());
    Shuffleboard.getTab("SHOOTER").add("Shooter Angle", this.getShooterAngle_deg());
  }

  // Returns the RPM of the shooter (ABS of Left and Right motor average)
  public int getShooterRpm() {
    return (int) (Math.abs(shooterMotorL.getEncoder().getVelocity()) +
        Math.abs(shooterMotorR.getEncoder().getVelocity())) / 2;
  }

  // returns false if the limit switch is pressed
  public boolean getLimitSwitch() {
    return backLimitSwitch.isPressed();
  }

  // Returns the angle of the shooter pivot (Right motor in deg)
  public double getShooterAngle_deg() {
    return shooterpivot_R_ABSEncoder.getPosition() * 360;
  }

  public Command setShooterFFvalue (Double ShooterFFValue) {
    return this.runOnce(() -> shooterpivotRPID.setFF(ShooterFFValue));
  }

  // Drives the shooter with given values from -1 to 1
  public void driveShooter(double input) {
    shooterMotorL.set(input);
    SmartDashboard.putNumber("Shooter power", input);
    SmartDashboard.putNumber("Shooter RPM", shooterMotorL.getEncoder().getVelocity());
    shooterMotorR.set(input);
  }

  // Drives Both Shooters at the same rpm using PID
  public void setshooterRPM(double setpoint) {
    RMPShooterLPid.setReference(setpoint, ControlType.kVelocity);
    RMPShooterRPid.setReference(setpoint, ControlType.kVelocity);
  }

  // Drives one shooter to a setpoint based on the boolean isLeftMotor
  public void setRPMshooteroneside(double setpoint, boolean isLeftMotor) {
    (isLeftMotor ? RMPShooterLPid : RMPShooterRPid).setReference(setpoint, ControlType.kVelocity);
  }

  // Drives the indexer with given values from -1 to 1
  public void setIndexer(double setpoint) {
    indexerMotorR.set(setpoint);
  }

  // Drives the pivot to a given angle in degrees
  public Command setShooterPivotangle(double setpoint_deg) {
    // shooterpivotLPID.setReference(setpoint, ControlType.kPosition);
    return this.run(() -> shooterpivotRPID.setReference(setpoint_deg / 360, ControlType.kPosition));
  }

  // Zeros the pivot -- call when laying flat
  public void zeroPivot() {
    shooterpivot_L_ABSEncoder.setZeroOffset(shooterpivot_L_ABSEncoder.getPosition());
    shooterpivot_R_ABSEncoder.setZeroOffset(shooterpivot_R_ABSEncoder.getPosition());
  }

  // puts the shooter motors in coast mode
  public void coastShooter() {
    shooterMotorL.setIdleMode(CANSparkMax.IdleMode.kCoast);
    shooterMotorR.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  // puts the shooter motors in brake mode
  public void brakeShooter() {
    shooterMotorL.setIdleMode(CANSparkMax.IdleMode.kBrake);
    shooterMotorR.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  // for testing
  // sets the pivot based on input -1 to 1
  public Command setShooterPivotraw(double input) {
    return this.runOnce(() -> shooterpivot_R.set(input));
  }
}
