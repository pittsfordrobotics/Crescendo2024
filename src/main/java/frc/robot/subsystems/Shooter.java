// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.DelayQueue;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DisabledInstantCommand;
import frc.robot.lib.FFCalculator;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private CANSparkFlex shooterMotorL;
  private CANSparkFlex shooterMotorR;
  private CANSparkMax indexerMotorL;
  private CANSparkMax indexerMotorR;
  private CANSparkMax shooterpivot_L;
  private CANSparkMax shooterpivot_R;
  private DigitalInput backLimitSwitch;

  private SparkPIDController shooterpivotRPID;
  // private SparkPIDController RPMShooterLPid;
  // private SparkPIDController RPMShooterRPid;
  private PIDController RPMShooterRPid;
  private PIDController RPMShooterLPid;

  private SparkAbsoluteEncoder shooterpivot_R_ABSEncoder;

  /** Creates a new Shooter. */
  public Shooter() {
    // // SHOOTER // //
    // Shooter Motor L
    shooterMotorL = new CANSparkFlex(ShooterConstants.CAN_SHOOTER_L, MotorType.kBrushless);
    shooterMotorL.restoreFactoryDefaults();
    shooterMotorL.setIdleMode(IdleMode.kCoast);
    shooterMotorL.setSmartCurrentLimit(40);
    
    // // Shooter L OnBoard PID -- Bad as of 2/5
    // RPMShooterLPid = shooterMotorL.getPIDController();
    // RPMShooterLPid.setFeedbackDevice(shooterMotorL.getEncoder());
    // RPMShooterLPid.setP(ShooterConstants.SHOOTER_P);
    // RPMShooterLPid.setI(ShooterConstants.SHOOTER_I);
    // RPMShooterLPid.setD(ShooterConstants.SHOOTER_D);

    // Shooter R OffBoard PID
    RPMShooterLPid = new PIDController(ShooterConstants.SHOOTER_P, ShooterConstants.SHOOTER_I,
        ShooterConstants.SHOOTER_D);
    RPMShooterLPid.setSetpoint(0);

    shooterMotorL.burnFlash();
    try {
      Thread.sleep(200);
    } catch (InterruptedException e) {
    }

    // Shooter Motor R
    shooterMotorR = new CANSparkFlex(ShooterConstants.CAN_SHOOTER_R, MotorType.kBrushless);
    shooterMotorR.restoreFactoryDefaults();
    shooterMotorR.setIdleMode(IdleMode.kCoast);
    shooterMotorR.setSmartCurrentLimit(40);
    shooterMotorR.setInverted(true);

    // // Shooter R OnBoard PID bad as of 2/5
    // RPMShooterRPid = shooterMotorR.getPIDController();
    // System.out.println(RPMShooterRPid.setFeedbackDevice(shooterMotorR.getEncoder()));
    // System.out.println(RPMShooterRPid.setP(ShooterConstants.SHOOTER_P));
    // System.out.println(RPMShooterRPid.setI(ShooterConstants.SHOOTER_I));
    // System.out.println(RPMShooterRPid.setD(ShooterConstants.SHOOTER_D));

    // Shooter R OffBoard PID
    RPMShooterRPid = new PIDController(ShooterConstants.SHOOTER_P, ShooterConstants.SHOOTER_I,
        ShooterConstants.SHOOTER_D);
    RPMShooterRPid.setSetpoint(0);

    shooterMotorR.burnFlash();
    try {
      Thread.sleep(200);
    } catch (InterruptedException e) {
    }

    // // INDEXER // //
    // Index Motor R (Leader)
    indexerMotorR = new CANSparkMax(ShooterConstants.CAN_INDEXER_R, MotorType.kBrushless);
    indexerMotorR.restoreFactoryDefaults();
    indexerMotorR.setSmartCurrentLimit(20);
    // Index Motor L
    indexerMotorL = new CANSparkMax(ShooterConstants.CAN_INDEXER_L, MotorType.kBrushless);
    indexerMotorL.restoreFactoryDefaults();
    indexerMotorL.setSmartCurrentLimit(20);
    indexerMotorL.follow(indexerMotorR, true);

    // // PIVOT // //
    // ShooterPivot R (Leader)
    shooterpivot_R = new CANSparkMax(ShooterConstants.CAN_SHOOTER_PIVOT_R, MotorType.kBrushless);
    shooterpivot_R.restoreFactoryDefaults();
    shooterpivot_R.setSmartCurrentLimit(40);
    shooterpivot_R.setInverted(true);
    shooterpivot_R.setIdleMode(CANSparkMax.IdleMode.kBrake);

    shooterpivot_R_ABSEncoder = shooterpivot_R.getAbsoluteEncoder(Type.kDutyCycle);
    shooterpivot_R_ABSEncoder.setInverted(true);
    shooterpivot_R_ABSEncoder.setPositionConversionFactor(360);
    // ShooterPivotPID R
    shooterpivotRPID = shooterpivot_R.getPIDController();
    shooterpivotRPID.setFeedbackDevice(shooterpivot_R_ABSEncoder);
    shooterpivotRPID.setP(ShooterConstants.SHOOTER_Pivot_P);
    shooterpivotRPID.setI(ShooterConstants.SHOOTER_Pivot_I);
    shooterpivotRPID.setD(ShooterConstants.SHOOTER_Pivot_D);
    // shooterpivotRPID.setOutputRange(-.05, .05);
    shooterpivot_R.burnFlash();
    // ShooterPivot L
    shooterpivot_L = new CANSparkMax(ShooterConstants.CAN_SHOOTER_PIVOT_L, MotorType.kBrushless);
    shooterpivot_L.restoreFactoryDefaults();
    shooterpivot_L.setSmartCurrentLimit(40);
    shooterpivot_L.setIdleMode(CANSparkMax.IdleMode.kBrake);

    shooterpivot_L.follow(shooterpivot_R, true);

    // Limit Switch
    backLimitSwitch = new DigitalInput(0);

    try {
      Thread.sleep(200);
    } catch (InterruptedException e) {
    }

    // // For PidTuningOnly
    // SmartDashboard.putNumber("Shooter Pivot P", shooterpivotRPID.getP());
    // SmartDashboard.putNumber("Shooter Pivot D", shooterpivotRPID.getD());
    // // // //

    Shuffleboard.getTab("SHOOTER").add("Shooter Pivot Set Coast", new DisabledInstantCommand(this::coastShooter));
    Shuffleboard.getTab("SHOOTER").add("Shooter Pivot Set Brake", new DisabledInstantCommand(this::brakeShooter));
    Shuffleboard.getTab("SHOOTER").add("Shooter Pivot Zero", new DisabledInstantCommand(this::zeroPivot));
    Shuffleboard.getTab("SHOOTER").addDouble("Shooter RPM", this::getShooterRpm);
    Shuffleboard.getTab("SHOOTER").addDouble("Shooter Angle", this::getShooterAngle_deg);
    Shuffleboard.getTab("SHOOTER").addBoolean("Shooter Limit Switch", this::getLimitSwitch);
  }

  @Override
  public void periodic() {
    // Shuffleboard.update();

    // Shooter OFFBOARD PID
    shooterMotorL.set(MathUtil.clamp(RPMShooterLPid.calculate(shooterMotorL.getEncoder().getVelocity()),-1,1));
    shooterMotorR.set(MathUtil.clamp(RPMShooterRPid.calculate(shooterMotorR.getEncoder().getVelocity()),-1,1));

    // // For PidTuningOnly
    // if (SmartDashboard.getNumber("Shooter Pivot P",
    // ShooterConstants.SHOOTER_Pivot_P) != shooterpivotRPID.getP()) {
    // shooterpivotRPID.setP(SmartDashboard.getNumber("Shooter Pivot P",
    // ShooterConstants.SHOOTER_Pivot_P));
    // }
    // if (SmartDashboard.getNumber("Shooter Pivot D",
    // ShooterConstants.SHOOTER_Pivot_D) != shooterpivotRPID.getD()) {
    // shooterpivotRPID.setD(SmartDashboard.getNumber("Shooter Pivot D",
    // ShooterConstants.SHOOTER_Pivot_D));
    // }
    // // //

    // shooterpivotRPID.setFF(FFCalculator.getInstance().calculateShooterFF());
  }

  // Returns the RPM of the shooter (ABS of Left and Rights motor average)
  public double getShooterRpm() {
    return shooterMotorR.getEncoder().getVelocity();
  }

  // returns true if the limit switch is pressed
  public boolean getLimitSwitch() {
    return backLimitSwitch.get();
  }

  // Returns the angle of the shooter pivot (Right motor in deg)
  public double getShooterAngle_deg() {
    return shooterpivot_R_ABSEncoder.getPosition();
  }

  public Command setShooterFFvalue(Double ShooterFFValue) {
    return this.runOnce(() -> shooterpivotRPID.setFF(ShooterFFValue));
  }

  // Drives both shooters to a common RPM setpoint
  public Command setshooterRPM(double setpoint) {
    return this.runOnce(() -> {
      // // onboard pid
      // RPMShooterLPid.setReference(setpoint, ControlType.kVelocity);
      // RPMShooterRPid.setReference(setpoint, ControlType.kVelocity);

      // offboard pid
      RPMShooterLPid.setSetpoint(setpoint);
      RPMShooterRPid.setSetpoint(setpoint);

      // // commands raw
      // shooterMotorR.set(setpoint);
      // shooterMotorL.set(setpoint);
    });
  }

  // // Drives one shooter to a setpoint based on the boolean isLeftMotor
  // public void setRPMshooteroneside(double setpoint, boolean isLeftMotor) {
  // (isLeftMotor ? RPMShooterLPid : RPMShooterRPid).setReference(setpoint,
  // ControlType.kVelocity);
  // }

  /** Drives the indexer with given values from -1 to 1 */
  public Command setIndexer(double setpoint) {
    return this.runOnce(() -> indexerMotorR.set(setpoint));
  }

  // Drives the pivot to a given angle in degrees
  public Command setShooterPivotangle(double setpoint_deg) {
    // shooterpivotLPID.setReference(setpoint, ControlType.kPosition);
    // return custom command with an execute and im finished
    // return this.runOnce(() -> shooterpivotRPID.setReference(setpoint_deg, ControlType.kPosition));
    double setpoint_deg_clamped = MathUtil.clamp(setpoint_deg, 2, 90);

    Command cmd = new RunCommand(() -> shooterpivotRPID.setReference(setpoint_deg_clamped, ControlType.kPosition, 0, FFCalculator.getInstance().calculateShooterFF()));
    cmd.until(() -> Math.abs(shooterpivot_R_ABSEncoder.getPosition() - setpoint_deg) < 2);
    return cmd;
  }

  // Zeros the pivot -- call when laying flat
  public void zeroPivot() {
    // shooterpivot_R_ABSEncoder.setZeroOffset(shooterpivot_R_ABSEncoder.getPosition());
    shooterpivot_R_ABSEncoder.setZeroOffset(MathUtil
        .inputModulus(shooterpivot_R_ABSEncoder.getPosition() + shooterpivot_R_ABSEncoder.getZeroOffset(), 0, 360));
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
