// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.DisabledInstantCommand;
import frc.robot.lib.FFCalculator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;


public class Intake extends SubsystemBase {

  private CANSparkMax intakeMotor;
  private CANSparkMax pivotMotorL;
  private CANSparkMax pivotMotorR;

  private SparkPIDController pivotRPID;

  private SparkAbsoluteEncoder pivotRABSEncoder;

  /** Creates a new Intake. */
  public Intake() {

    // Intake Pivot Motor R (Leader)
    pivotMotorR = new CANSparkMax(IntakeConstants.CAN_INTAKE_PIVOT_R, MotorType.kBrushless);
    pivotMotorR.restoreFactoryDefaults();
    pivotMotorR.setSmartCurrentLimit(40);
    pivotMotorR.setInverted(true);
    pivotRABSEncoder = pivotMotorR.getAbsoluteEncoder(Type.kDutyCycle);
    pivotRABSEncoder.setPositionConversionFactor(360);
    // Intake Pivot Pid (in the right motor controller)
    pivotRPID = pivotMotorR.getPIDController();
    pivotRPID.setFeedbackDevice(pivotRABSEncoder);
    pivotRPID.setP(IntakeConstants.INTAKE_Pivot_P);
    pivotRPID.setI(IntakeConstants.INTAKE_Pivot_I);
    pivotRPID.setD(IntakeConstants.INTAKE_Pivot_D);
    pivotRPID.setPositionPIDWrappingEnabled(true);
    pivotRPID.setPositionPIDWrappingMaxInput(360);
    pivotRPID.setPositionPIDWrappingMinInput(0);
    pivotRPID.setOutputRange(-0.6, .6);

    pivotMotorR.burnFlash();
    try {
      Thread.sleep(200);
    } catch (InterruptedException e) {
    }

    // Intake Pivot Motor L
    pivotMotorL = new CANSparkMax(IntakeConstants.CAN_INTAKE_PIVOT_L, MotorType.kBrushless);
    pivotMotorL.restoreFactoryDefaults();
    pivotMotorL.setSmartCurrentLimit(40);
    pivotMotorL.follow(pivotMotorR, true);

    pivotMotorL.burnFlash();
    try {
      Thread.sleep(200);
    } catch (InterruptedException e) {
    }

    // Intake Motor
    intakeMotor = new CANSparkMax(IntakeConstants.CAN_INTAKE, MotorType.kBrushless);
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(true);
    intakeMotor.setSmartCurrentLimit(20);

    intakeMotor.burnFlash();
    try {
      Thread.sleep(200);
    } catch (InterruptedException e) {
    }

    // For PidTuningOnly
    SmartDashboard.putNumber("Intake P", pivotRPID.getP());
    SmartDashboard.putNumber("Intake D", pivotRPID.getD());

    
    Shuffleboard.getTab("Intake").addDouble("Intake RPM", this::getIntakeRPM);
    Shuffleboard.getTab("Intake").addDouble("Intake Pivot Angle", this::getIntakePivotAngleDeg);

    Shuffleboard.getTab("Intake").add("Zero Intake Pivot", new DisabledInstantCommand(this::zeroIntakePivot));

    Shuffleboard.getTab("Intake").add("Intake Pivot Coast", new DisabledInstantCommand(this::setPivotCoastCommand));
    Shuffleboard.getTab("Intake").add("Intake Pivot Brake", new DisabledInstantCommand(this::setPivotBrakeCommand));
  }

  @Override
  public void periodic() {
    Shuffleboard.update();

    // For PidTuningOnly
    if (SmartDashboard.getNumber("Intake P", IntakeConstants.INTAKE_Pivot_P) != pivotRPID.getP()) {
      pivotRPID.setP(SmartDashboard.getNumber("Intake P",
          IntakeConstants.INTAKE_Pivot_P));
    }
    if (SmartDashboard.getNumber("Intake D", IntakeConstants.INTAKE_Pivot_D) != pivotRPID.getD()) {
      pivotRPID.setD(SmartDashboard.getNumber("Intake D",
          IntakeConstants.INTAKE_Pivot_D));
    }
    // //
  }

  // Gets the RPM of the intake motor
  public double getIntakeRPM() {
    return intakeMotor.getEncoder().getVelocity();
  }

  // Gets the Angle of the intake pivot in degrees
  public double getIntakePivotAngleDeg() {
    return pivotRABSEncoder.getPosition();
  }

  // Sets the intake pivot FF values
  public Command setIntakeFFCommand(double IntakeFFValue) {
    return this.runOnce(() -> pivotRPID.setFF(IntakeFFValue));
  }

  // Zeros the Intake Angle
  public void zeroIntakePivot() {
    pivotRABSEncoder.setZeroOffset(MathUtil
        .inputModulus(pivotRABSEncoder.getPosition() + pivotRABSEncoder.getZeroOffset(), 0, 360));
    pivotMotorR.burnFlash();
  }

  // Sets the intake pivot to coast
  public Command setPivotCoastCommand() {
    return this.runOnce(() -> pivotMotorR.setIdleMode(CANSparkMax.IdleMode.kCoast));
  }

  // Sets the intake pivot to brake
  public Command setPivotBrakeCommand() {
    return this.runOnce(() -> pivotMotorR.setIdleMode(CANSparkMax.IdleMode.kBrake));
  }

  // Sets the intake rpm to a certain value from -1 to 1
  public Command spinIntakeCommand(double input) {
    return this.runOnce(() -> intakeMotor.set(input));
  }

  // Sets the intake pivot angle to a certain angle using PID on right motors
  // **set in degrees**
  public Command setPivotAngleCommand(double setpoint_deg) {
    double setpoint_deg_clamped = MathUtil.clamp(setpoint_deg,0,170);
    Command cmd = new RunCommand(() -> pivotRPID.setReference(setpoint_deg_clamped, ControlType.kPosition, 0, FFCalculator.getInstance().calculateIntakeFF()), this);
    return cmd;
  }

//  // For Testing
//  public Command intakePivotRaw(double input_test) {
//    return this.runOnce(() -> pivotMotorR.set(input_test));
//  }
  
  // Sets the intake pivot angle to a certain angle using a supplier
  public Command setPivotAngleSupplier(DoubleSupplier intakePivotAngle) {
    double intakePivotAngle_clamped = MathUtil.clamp(intakePivotAngle.getAsDouble(),0,170);
    return this.runOnce(() -> pivotRPID.setReference(intakePivotAngle_clamped, ControlType.kPosition, 0, FFCalculator.getInstance().calculateIntakeFF()));
  }}