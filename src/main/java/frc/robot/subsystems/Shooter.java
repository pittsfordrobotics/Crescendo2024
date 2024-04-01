// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DisabledInstantCommand;
import frc.robot.lib.FFCalculator;
import frc.robot.lib.util.ShooterInterpolationHelper;

public class Shooter extends SubsystemBase {
  private CANSparkMax shooterMotorL;
  private CANSparkMax shooterMotorR;
  private CANSparkMax indexerMotorL;
  private CANSparkMax indexerMotorR;
  private CANSparkMax pivotMotorL;
  private CANSparkMax pivotMotorR;
  private DigitalInput backLimitSwitch;

  private SparkPIDController pivotRPID;
  private SparkPIDController shooterRPID;
  private SparkPIDController shooterLPID;

  private SparkAbsoluteEncoder pivotRABSEncoder;
  private double pivotAngleSetpointDeg;
  private double targetPivotAngleSetpointDeg;
  private double shooterRPMSetpoint;
  private SlewRateLimiter pivotTargetAngleRateLimiter;

  /** Creates a new Shooter. */
  public Shooter() {
    pivotAngleSetpointDeg = RobotConstants.STORED_ShooterPivotAngle;
    // // SHOOTER // //
    // Shooter Motor L
    shooterMotorL = new CANSparkMax(ShooterConstants.CAN_SHOOTER_L, MotorType.kBrushless);
    shooterMotorL.restoreFactoryDefaults();
    shooterMotorL.setIdleMode(IdleMode.kCoast);
    shooterMotorL.setSmartCurrentLimit(45);
    // Shooter left pid
    shooterLPID = shooterMotorL.getPIDController();
    shooterLPID.setFeedbackDevice(shooterMotorL.getEncoder());
    shooterLPID.setP(ShooterConstants.SHOOTER_P);
    shooterLPID.setI(ShooterConstants.SHOOTER_I);
    shooterLPID.setD(ShooterConstants.SHOOTER_D);
    shooterLPID.setFF(ShooterConstants.SHOOTER_L_FFGain);

    shooterMotorL.burnFlash();

    // Shooter Motor R
    shooterMotorR = new CANSparkMax(ShooterConstants.CAN_SHOOTER_R, MotorType.kBrushless);
    shooterMotorR.restoreFactoryDefaults();
    shooterMotorR.setIdleMode(IdleMode.kCoast);
    shooterMotorR.setSmartCurrentLimit(45);
    shooterMotorR.setInverted(true);
    // Shooter right pid
    shooterRPID = shooterMotorR.getPIDController();
    shooterRPID.setFeedbackDevice(shooterMotorR.getEncoder());
    shooterRPID.setP(ShooterConstants.SHOOTER_P);
    shooterRPID.setI(ShooterConstants.SHOOTER_I);
    shooterRPID.setD(ShooterConstants.SHOOTER_D);
    shooterRPID.setFF(ShooterConstants.SHOOTER_R_FFGain);

    shooterMotorR.burnFlash();

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
    pivotMotorR = new CANSparkMax(ShooterConstants.CAN_SHOOTER_PIVOT_R, MotorType.kBrushless);
    pivotMotorR.restoreFactoryDefaults();
    pivotMotorR.setSmartCurrentLimit(40);
    pivotMotorR.setInverted(true);
    pivotMotorR.setIdleMode(CANSparkMax.IdleMode.kBrake);
    pivotRABSEncoder = pivotMotorR.getAbsoluteEncoder(Type.kDutyCycle);
    pivotRABSEncoder.setInverted(true);
    pivotRABSEncoder.setPositionConversionFactor(360);
    // ShooterPivotPID R
    pivotRPID = pivotMotorR.getPIDController();
    pivotRPID.setFeedbackDevice(pivotRABSEncoder);
    pivotRPID.setP(ShooterConstants.SHOOTER_Pivot_P);
    pivotRPID.setI(ShooterConstants.SHOOTER_Pivot_I);
    pivotRPID.setD(ShooterConstants.SHOOTER_Pivot_D);
    pivotRPID.setOutputRange(-0.4, 0.4);
    pivotRPID.setPositionPIDWrappingEnabled(true);
    pivotRPID.setPositionPIDWrappingMaxInput(360);
    pivotRPID.setPositionPIDWrappingMinInput(0);
    pivotMotorR.setOpenLoopRampRate(0.25);

    pivotMotorR.burnFlash();
    try {
      Thread.sleep(200);
    } catch (InterruptedException e) {
    }

    // ShooterPivot L
    pivotMotorL = new CANSparkMax(ShooterConstants.CAN_SHOOTER_PIVOT_L, MotorType.kBrushless);
    pivotMotorL.restoreFactoryDefaults();
    pivotMotorL.setSmartCurrentLimit(40);
    pivotMotorL.setIdleMode(CANSparkMax.IdleMode.kBrake);
    pivotMotorL.follow(pivotMotorR, true);

    // Limit Switch
    backLimitSwitch = new DigitalInput(0);

    try {
      Thread.sleep(200);
    } catch (InterruptedException e) {
    }

    // Logging
    // SmartDashboard.putNumber("Shooter Pivot P", pivotRPID.getP());
    // SmartDashboard.putNumber("Shooter Pivot D", pivotRPID.getD());

    // SmartDashboard.putNumber("ShooterRight FF Gain", shooterRPID.getFF());
    // SmartDashboard.putNumber("ShooterLeft FF Gain", shooterLPID.getFF());

    // SmartDashboard.putNumber("Shooter L&R P", shooterRPID.getP());

    Shuffleboard.getTab("SHOOTER").add(this);

    Shuffleboard.getTab("COMP").addDouble("Shooter RPM R", this::getShooterRRPM);
    Shuffleboard.getTab("COMP").addDouble("Shooter RPM L", this::getShooterLRPM);

    Shuffleboard.getTab("SHOOTER").addDouble("Shooter Angle", this::getPivotAngleDeg);
    Shuffleboard.getTab("COMP").addBoolean("Shooter Limit Switch", this::getLimitSwitch);

    // Shuffleboard.getTab("SHOOTER").add("Shooter Pivot Set Coast", new
    // DisabledInstantCommand(this::coastShooter));
    // Shuffleboard.getTab("SHOOTER").add("Shooter Pivot Set Brake", new
    // DisabledInstantCommand(this::brakeShooter));

    Shuffleboard.getTab("SHOOTER").add("Shooter Pivot Zero", new DisabledInstantCommand(this::zeroPivot));

  }

  @Override
  public void periodic() {
    pivotRPID.setReference(pivotAngleSetpointDeg, ControlType.kPosition, 0,
        FFCalculator.getInstance().calculateShooterFF());

    // // For PidTuningOnly
    // if (SmartDashboard.getNumber("Shooter Pivot P",
    // ShooterConstants.SHOOTER_Pivot_P) != pivotRPID.getP()) {
    // pivotRPID.setP(SmartDashboard.getNumber("Shooter Pivot P",
    // ShooterConstants.SHOOTER_Pivot_P));
    // }
    // if (SmartDashboard.getNumber("Shooter Pivot D",
    // ShooterConstants.SHOOTER_Pivot_D) != pivotRPID.getD()) {
    // pivotRPID.setD(SmartDashboard.getNumber("Shooter Pivot D",
    // ShooterConstants.SHOOTER_Pivot_D));
    // }

    // if (SmartDashboard.getNumber("ShooterRight FF Gain",
    // ShooterConstants.SHOOTER_L_FFGain) != shooterRPID.getFF()) {
    // shooterRPID.setFF(SmartDashboard.getNumber("ShooterRight FF Gain",
    // ShooterConstants.SHOOTER_L_FFGain));
    // }

    // if (SmartDashboard.getNumber("ShooterLeft FF Gain",
    // ShooterConstants.SHOOTER_R_FFGain) != shooterLPID.getFF()) {
    // shooterLPID.setFF(SmartDashboard.getNumber("ShooterLeft FF Gain",
    // ShooterConstants.SHOOTER_R_FFGain));
    // }

    // if (SmartDashboard.getNumber("Shooter L&R P",
    // ShooterConstants.SHOOTER_P) != shooterRPID.getP()) {
    // shooterRPID.setP(SmartDashboard.getNumber("Shooter L&R P",
    // ShooterConstants.SHOOTER_P));
    // shooterLPID.setP(SmartDashboard.getNumber("Shooter L&R P",
    // ShooterConstants.SHOOTER_P));
    // }
    // //

  }

  // Returns the RPM of the right shooter
  public double getShooterRRPM() {
    return shooterMotorR.getEncoder().getVelocity();
  }

  // Returns the RPM of the left shooter
  public double getShooterLRPM() {
    return shooterMotorL.getEncoder().getVelocity();
  }

  /** returns true if the limit switch is pressed */
  public boolean getLimitSwitch() {
    return backLimitSwitch.get();
  }

  // Returns the angle of the shooter pivot (Right motor in deg)
  public double getPivotAngleDeg() {
    return pivotRABSEncoder.getPosition();
  }

  public double getPivotAngleSetpointDeg() {
    return pivotAngleSetpointDeg;
  }

  // Drives both shooters to a common RPM setpoint
  public Command setShooterRPMCommand(double setpointRPM) {
    return this.runOnce(() -> {
      // // onboard pid
      shooterRPMSetpoint = setpointRPM;
      shooterRPID.setReference(setpointRPM, ControlType.kVelocity);
      shooterLPID.setReference(setpointRPM, ControlType.kVelocity);

      // // commands raw
      // shooterMotorR.set(setpoint);
      // shooterMotorL.set(setpoint);
    });
  }

  public Command setShooterRPMCommand(DoubleSupplier setpointRPM) {
    return this.runOnce(() -> {
      // // onboard pid
      shooterRPMSetpoint = setpointRPM.getAsDouble();
      shooterRPID.setReference(setpointRPM.getAsDouble(), ControlType.kVelocity);
      shooterLPID.setReference(setpointRPM.getAsDouble(), ControlType.kVelocity);

      // // commands raw
      // shooterMotorR.set(setpoint);
      // shooterMotorL.set(setpoint);
    });
  }

  public Command spinShooterCommand(double output) {
    return this.run(() -> {
      shooterMotorL.set(output);
      shooterMotorR.set(output);
    });
  }

  public Command waitForShooterRPMCommand() {
    Command cmd = new WaitUntilCommand(() -> getShooterRRPM() > shooterRPMSetpoint - 50 &&
        getShooterLRPM() > shooterRPMSetpoint - 50);
    cmd.addRequirements(this);
    return cmd;
  }

  public Command waitForLimitSwitchCommand() {
    Command cmd = new WaitUntilCommand(this::getLimitSwitch);
    cmd.addRequirements(this);
    return cmd;
  }

  // Drives the indexer with given values from -1 to 1
  public Command spinIndexerCommand(double setpoint) {
    return this.runOnce(() -> indexerMotorR.set(setpoint));
  }

  // Drives the pivot to a given angle in degrees
  public Command setPivotAngleCommand(double setpoint_deg) {
    double setpoint_deg_clamped = MathUtil.clamp(setpoint_deg, 0, 90);
    return this.runOnce(() -> pivotAngleSetpointDeg = setpoint_deg_clamped);
  }



  public Command setPivotAngleCommand(DoubleSupplier setpoint_deg) {
    return this.runOnce(() -> {
      double setpoint_deg_clamped = MathUtil.clamp(setpoint_deg.getAsDouble(), 0, 90);
      pivotAngleSetpointDeg = setpoint_deg_clamped;
    });
  }

  // Requiring the intake in the shooter subsystem?? Probly better to just use
  // common speaker command in robot container - evan
  public Command autoAimSpeaker(DoubleSupplier distanceSupplier, Intake intake) {
    return new ParallelCommandGroup(
        this.setPivotAngleCommand(ShooterInterpolationHelper.getShooterAngle(distanceSupplier.getAsDouble())),
        this.setShooterRPMCommand(ShooterInterpolationHelper.getShooterRPM(distanceSupplier.getAsDouble())))
        .beforeStarting(
            new SequentialCommandGroup(
                intake.setPivotAngleCommand(RobotConstants.SUBWOOF_IntakePivotAngle),
                intake.waitForPivotAngleCommand()));
  }

  public Command waitForPivotAngleCommand(double degTolerance) {
    Command cmd = new WaitUntilCommand(() -> Math.abs(getPivotAngleDeg() - getPivotAngleSetpointDeg()) < degTolerance);
    cmd.addRequirements(this);
    return cmd;
  }

  public Command waitForPivotAngleCommand() {
    return waitForPivotAngleCommand(7.5);
  }

  // Zeros the pivot -- call when laying flat
  public void zeroPivot() {
    pivotRABSEncoder.setZeroOffset(MathUtil
        .inputModulus(pivotRABSEncoder.getPosition() + pivotRABSEncoder.getZeroOffset(), 0, 360));
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

  // // // for testing
  //
  // // sets the pivot based on input -1 to 1
  // public Command setShooterPivotraw(double input) {
  // return this.runOnce(() -> pivotMotorR.set(input));
  // }

  // Drives both shooters to a common RPM setpoint using a supplier
  public Command setShooterRPMSupplierCommand() {
    return this.runOnce(() -> {
      // // onboard pid
      shooterLPID.setReference(SmartDashboard.getNumber("ShooterRPM_CHANGEME", 0), ControlType.kVelocity);
      shooterRPID.setReference(SmartDashboard.getNumber("ShooterRPM_CHANGEME", 0), ControlType.kVelocity);

      // // commands raw
      // shooterMotorR.set(setpoint.getAsDouble());
      // shooterMotorL.set(setpoint.getAsDouble());
    });
  }

  // Drives the pivot to a angle using a double suplier (same as above just using
  // a supplier)
  public Command setPivotAngleSupplierCommand() {
    return this.runOnce(() -> {
      double setpoint = SmartDashboard.getNumber("ShooterPivotAngle_CHANGEME", 20);
      double setpointDegClamped = MathUtil.clamp(setpoint, 0, 90);
      pivotAngleSetpointDeg = setpointDegClamped;
    });
  }
}
