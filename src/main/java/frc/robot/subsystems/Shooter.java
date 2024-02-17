// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DisabledInstantCommand;
import frc.robot.lib.FFCalculator;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;


public class Shooter extends SubsystemBase {
  private CANSparkFlex shooterMotorL;
  private CANSparkFlex shooterMotorR;
  private CANSparkMax indexerMotorL;
  private CANSparkMax indexerMotorR;
  private CANSparkMax shooterpivot_L;
  private CANSparkMax shooterpivot_R;
  private DigitalInput backLimitSwitch;

  private SparkPIDController shooterpivotRPID;
  private SparkPIDController RPMShooterLPid;
  private SparkPIDController RPMShooterRPid;
  // private PIDController RPMShooterRPid;
  // private PIDController RPMShooterLPid;

  private SparkAbsoluteEncoder shooterpivot_R_ABSEncoder;

  /** Creates a new Shooter. */
  public Shooter() {
    // // SHOOTER // // OnBoard PID -- Fine but right motor dies sometimes (not
    // substantial or impactfull - 2/14)
    // Shooter Motor L
    shooterMotorL = new CANSparkFlex(ShooterConstants.CAN_SHOOTER_L, MotorType.kBrushless);
    shooterMotorL.restoreFactoryDefaults();
    shooterMotorL.setIdleMode(IdleMode.kCoast);
    shooterMotorL.setSmartCurrentLimit(40);
    // Shooter left pid
    RPMShooterLPid = shooterMotorL.getPIDController();
    RPMShooterLPid.setFeedbackDevice(shooterMotorL.getEncoder());
    RPMShooterLPid.setP(ShooterConstants.SHOOTER_P);
    RPMShooterLPid.setI(ShooterConstants.SHOOTER_I);
    RPMShooterLPid.setD(ShooterConstants.SHOOTER_D);
    RPMShooterLPid.setFF(ShooterConstants.SHOOTER_L_FFGain);

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
    // Shooter right pid
    RPMShooterRPid = shooterMotorR.getPIDController();
    RPMShooterRPid.setFeedbackDevice(shooterMotorR.getEncoder());
    RPMShooterRPid.setP(ShooterConstants.SHOOTER_P);
    RPMShooterRPid.setI(ShooterConstants.SHOOTER_I);
    RPMShooterRPid.setD(ShooterConstants.SHOOTER_D);
    RPMShooterRPid.setFF(ShooterConstants.SHOOTER_R_FFGain);

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
    // ShooterPivot Right (Leader)
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
    shooterpivotRPID.setOutputRange(-0.4, 0.4);
    shooterpivotRPID.setPositionPIDWrappingEnabled(true);
    shooterpivotRPID.setPositionPIDWrappingMaxInput(360);
    shooterpivotRPID.setPositionPIDWrappingMinInput(0);

    shooterpivot_R.burnFlash();
    try {
      Thread.sleep(200);
    } catch (InterruptedException e) {
    }

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

    // Logging
    SmartDashboard.putNumber("Shooter Pivot P", shooterpivotRPID.getP());
    SmartDashboard.putNumber("Shooter Pivot D", shooterpivotRPID.getD());

    Shuffleboard.getTab("SHOOTER").add(this);

    Shuffleboard.getTab("SHOOTER").addDouble("Shooter RPM R", this::getShooterRpm_R);
    Shuffleboard.getTab("SHOOTER").addDouble("Shooter RPM L", this::getShooterRpm_L);

    Shuffleboard.getTab("SHOOTER").addDouble("Shooter Angle", this::getShooterAngle_deg);
    Shuffleboard.getTab("SHOOTER").addBoolean("Shooter Limit Switch", this::getLimitSwitch);

    Shuffleboard.getTab("SHOOTER").add("Shooter Pivot Set Coast", new DisabledInstantCommand(this::coastShooter));
    Shuffleboard.getTab("SHOOTER").add("Shooter Pivot Set Brake", new DisabledInstantCommand(this::brakeShooter));

    Shuffleboard.getTab("SHOOTER").add("Shooter Pivot Zero", new DisabledInstantCommand(this::zeroPivot));

  }

  @Override
  public void periodic() {
    Shuffleboard.update();

    // For PidTuningOnly
    if (SmartDashboard.getNumber("Shooter Pivot P",
        ShooterConstants.SHOOTER_Pivot_P) != shooterpivotRPID.getP()) {
      shooterpivotRPID.setP(SmartDashboard.getNumber("Shooter Pivot P",
          ShooterConstants.SHOOTER_Pivot_P));
    }
    if (SmartDashboard.getNumber("Shooter Pivot D",
        ShooterConstants.SHOOTER_Pivot_D) != shooterpivotRPID.getD()) {
      shooterpivotRPID.setD(SmartDashboard.getNumber("Shooter Pivot D",
          ShooterConstants.SHOOTER_Pivot_D));
    }
    // //

  }

  // Returns the RPM of the right shooter
  public double getShooterRpm_R() {
    return shooterMotorR.getEncoder().getVelocity();
  }

  // Returns the RPM of the left shooter
  public double getShooterRpm_L() {
    return shooterMotorL.getEncoder().getVelocity();
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
      RPMShooterLPid.setReference(setpoint, ControlType.kVelocity);
      RPMShooterRPid.setReference(setpoint, ControlType.kVelocity);

      // // commands raw
      // shooterMotorR.set(setpoint);
      // shooterMotorL.set(setpoint);
    });
  }

  // Drives the indexer with given values from -1 to 1
  public Command setIndexer(double setpoint) {
    return this.runOnce(() -> indexerMotorR.set(setpoint));
  }

  // Drives the pivot to a given angle in degrees
  public Command setShooterPivotangle(double setpoint_deg) {
    double setpoint_deg_clamped = MathUtil.clamp(setpoint_deg, 0, 90);
    Command cmd = Commands.run(() -> shooterpivotRPID.setReference(setpoint_deg_clamped, ControlType.kPosition, 0,
        FFCalculator.getInstance().calculateShooterFF()), this);
    return cmd;
  }

  // Zeros the pivot -- call when laying flat
  public void zeroPivot() {
    shooterpivot_R_ABSEncoder.setZeroOffset(MathUtil
        .inputModulus(shooterpivot_R_ABSEncoder.getPosition() + shooterpivot_R_ABSEncoder.getZeroOffset(), 0, 360));
    shooterMotorR.burnFlash();
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

  // // for testing

  // sets the pivot based on input -1 to 1
  public Command setShooterPivotraw(double input) {
    return this.runOnce(() -> shooterpivot_R.set(input));
  }

  // Drives both shooters to a common RPM setpoint using a supplier
  public Command setshooterRPMSupplier(DoubleSupplier setpoint) {
    return this.runOnce(() -> {
      // // onboard pid
      RPMShooterLPid.setReference(setpoint.getAsDouble(), ControlType.kVelocity);
      RPMShooterRPid.setReference(setpoint.getAsDouble(), ControlType.kVelocity);

      // // commands raw
      // shooterMotorR.set(setpoint.getAsDouble());
      // shooterMotorL.set(setpoint.getAsDouble());
    });
  }

  // Drives the pivot to a angle using a double suplier (same as above just using a supplier)
  public Command setShooterPivotangleSupplier(DoubleSupplier setpoint_deg) {
    double setpoint_deg_clamped = MathUtil.clamp(setpoint_deg.getAsDouble(), 0, 90);
    return this.runOnce(() -> shooterpivotRPID.setReference(setpoint_deg_clamped, ControlType.kPosition, 0,
        FFCalculator.getInstance().calculateShooterFF()));
  }

}
