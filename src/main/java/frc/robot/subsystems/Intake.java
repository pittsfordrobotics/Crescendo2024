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
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.DisabledInstantCommand;
import frc.robot.lib.FFCalculator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;


public class Intake extends SubsystemBase {

  private CANSparkMax intakeMotor;
  private CANSparkMax pivotMotorL;
  private CANSparkMax pivotMotorR;

  private SparkPIDController pivotRPID;

  private SparkAbsoluteEncoder pivotRABSEncoder;
  private double pivotAngleSetpointDeg;

  /** Creates a new Intake. */
  public Intake() {
    pivotAngleSetpointDeg = RobotConstants.STORED_IntakePivotAngle;
        // Intake Pivot Motor R (Leader)
    pivotMotorR = new CANSparkMax(IntakeConstants.CAN_INTAKE_PIVOT_R, MotorType.kBrushless);
    pivotMotorR.restoreFactoryDefaults();
    pivotMotorR.setSmartCurrentLimit(40);
    pivotMotorR.setInverted(true);
    pivotMotorR.setOpenLoopRampRate(0.25);
    pivotRABSEncoder = pivotMotorR.getAbsoluteEncoder(Type.kDutyCycle);
    pivotRABSEncoder.setPositionConversionFactor(360);
    pivotRABSEncoder.setInverted(true);
    // Intake Pivot Pid (in the right motor controller)
    pivotRPID = pivotMotorR.getPIDController();
    pivotRPID.setFeedbackDevice(pivotRABSEncoder);
    pivotRPID.setP(IntakeConstants.INTAKE_Pivot_P);
    pivotRPID.setI(IntakeConstants.INTAKE_Pivot_I);
    pivotRPID.setD(IntakeConstants.INTAKE_Pivot_D);
    pivotRPID.setPositionPIDWrappingEnabled(true);
    pivotRPID.setPositionPIDWrappingMaxInput(360);
    pivotRPID.setPositionPIDWrappingMinInput(0);
    pivotRPID.setOutputRange(-0.8, .8);

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
    resetintakemotor();
    try {
      Thread.sleep(200);
    } catch (InterruptedException e) {
    }
    resetintakemotor();
    try {
      Thread.sleep(200);
    } catch (InterruptedException e) {
    }
    resetintakemotor();
    try {
      Thread.sleep(200);
    } catch (InterruptedException e) {
    }
    // intakeMotor.setOpenLoopRampRate(2); // sets minimum time in seconds that the motor takes to go from 0 to full throttle
    // TODO: Tune this ramp rate

    // For PidTuningOnly
    // SmartDashboard.putNumber("Intake P", pivotRPID.getP());
    // SmartDashboard.putNumber("Intake D", pivotRPID.getD());

    
    Shuffleboard.getTab("Intake").addDouble("Intake RPM", this::getIntakeRPM);
    Shuffleboard.getTab("Intake").addDouble("Intake Pivot Angle", this::getPivotAngleDeg);

    Shuffleboard.getTab("Intake").add("Zero Intake Pivot", new DisabledInstantCommand(this::zeroIntakePivot));
    Shuffleboard.getTab("COMP").add("Reset Intake Motor", new DisabledInstantCommand(this::resetintakemotor));

    // Shuffleboard.getTab("Intake").add("Intake Pivot Coast", new DisabledInstantCommand(this::setPivotCoastCommand));
    // Shuffleboard.getTab("Intake").add("Intake Pivot Brake", new DisabledInstantCommand(this::setPivotBrakeCommand));
  }

  private void resetintakemotor() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(true);
    intakeMotor.setSmartCurrentLimit(20);
    intakeMotor.setOpenLoopRampRate(0.5);
    intakeMotor.burnFlash();
  }
  
  public Command resetIntakeMotor (){
    return this.runOnce(() -> resetintakemotor());
  }

  @Override
  public void periodic() {
    pivotRPID.setReference(pivotAngleSetpointDeg, ControlType.kPosition, 0, FFCalculator.getInstance().calculateIntakeFF());

    // // For PidTuningOnly
    // if (SmartDashboard.getNumber("Intake P", IntakeConstants.INTAKE_Pivot_P) != pivotRPID.getP()) {
    //   pivotRPID.setP(SmartDashboard.getNumber("Intake P",
    //       IntakeConstants.INTAKE_Pivot_P));
    // }
    // if (SmartDashboard.getNumber("Intake D", IntakeConstants.INTAKE_Pivot_D) != pivotRPID.getD()) {
    //   pivotRPID.setD(SmartDashboard.getNumber("Intake D",
    //       IntakeConstants.INTAKE_Pivot_D));
    // }
    // // //
  }

  // Gets the RPM of the intake motor
  public double getIntakeRPM() {
    return intakeMotor.getEncoder().getVelocity();
  }

  // Gets the Angle of the intake pivot in degrees
  public double getPivotAngleDeg() {
    return pivotRABSEncoder.getPosition();
  }
  public double getPivotAngleSetpointDeg() {
    return pivotAngleSetpointDeg;
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
  public Command setPivotAngleCommand(double setpointDeg) {
    double setpointDegClamped = MathUtil.clamp(setpointDeg,0,170);
    return this.runOnce(() -> pivotAngleSetpointDeg = setpointDegClamped);
  }
  public Command waitForPivotAngleCommand(double degTolerance) {
    Command cmd = new WaitUntilCommand(() -> Math.abs(getPivotAngleDeg() - getPivotAngleSetpointDeg()) < degTolerance);
    cmd.addRequirements(this);
    return cmd;
  }

  public Command waitForPivotAngleCommand() {
    return waitForPivotAngleCommand(15);
  }

//  // For Testing
//  public Command intakePivotRaw(double input_test) {
//    return this.runOnce(() -> pivotMotorR.set(input_test));
//  }
  
  // Sets the intake pivot angle to a certain angle using a supplier
  public Command setPivotAngleSupplierCommand() {
    return this.runOnce(() -> {
    double setpoint = SmartDashboard.getNumber("IntakePivotAngle_CHANGEME", 170);
    double setpointDegClamped = MathUtil.clamp(setpoint,0,170);   
    pivotAngleSetpointDeg = setpointDegClamped;});
    
  }}