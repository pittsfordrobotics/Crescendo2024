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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private CANSparkMax intakeMotor;
  private CANSparkMax intakePivotMotorL;
  private CANSparkMax intakePivotMotorR;

  private SparkPIDController intakepivotPIDR;

  private SparkAbsoluteEncoder intakePivotABSEncoderR;

  /** Creates a new Intake. */
  public Intake() {

    // Intake Pivot Motor R (Leader)
    intakePivotMotorR = new CANSparkMax(IntakeConstants.CAN_INTAKE_PIVOT_R, MotorType.kBrushless);
    intakePivotMotorR.restoreFactoryDefaults();
    intakePivotMotorR.setSmartCurrentLimit(20);
    intakePivotABSEncoderR = intakePivotMotorR.getAbsoluteEncoder(Type.kDutyCycle);
    intakePivotABSEncoderR.setPositionConversionFactor(360);

    // Intake Pivot Pid (in the right motor controller)
    intakepivotPIDR = intakePivotMotorR.getPIDController();
    intakepivotPIDR.setFeedbackDevice(intakePivotABSEncoderR);
    intakepivotPIDR.setP(IntakeConstants.INTAKE_Pivot_P);
    intakepivotPIDR.setI(IntakeConstants.INTAKE_Pivot_I);
    intakepivotPIDR.setD(IntakeConstants.INTAKE_Pivot_D);

    intakePivotMotorR.burnFlash();
    try {
      Thread.sleep(200);
    } catch (InterruptedException e) {
    }

    // Intake Pivot Motor L
    intakePivotMotorL = new CANSparkMax(IntakeConstants.CAN_INTAKE_PIVOT_L, MotorType.kBrushless);
    intakePivotMotorL.restoreFactoryDefaults();
    intakePivotMotorL.setSmartCurrentLimit(20);
    intakePivotMotorL.follow(intakePivotMotorR, true);

    intakePivotMotorL.burnFlash();
    try {
      Thread.sleep(200);
    } catch (InterruptedException e) {
    }

    // Intake Motor
    intakeMotor = new CANSparkMax(IntakeConstants.CAN_INTAKE, MotorType.kBrushless);
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setSmartCurrentLimit(20);

    intakeMotor.burnFlash();
    try {
      Thread.sleep(200);
    } catch (InterruptedException e) {
    }

    // For PidTuningOnly
    SmartDashboard.putNumber("Intake P", intakepivotPIDR.getP());
    SmartDashboard.putNumber("Intake D", intakepivotPIDR.getD());
    // // //

    // Zeroed at intake position
    Shuffleboard.getTab("Intake").add("Zero Intake Pivot", new DisabledInstantCommand(this::zeroIntakePivot));
    Shuffleboard.getTab("Intake").add("Intake RPM", this.getIntakeRpm());
    Shuffleboard.getTab("Intake").add("Intake Pivot Angle", this.getIntakePivotAngle_deg());
  }

  @Override
  public void periodic() {
    Shuffleboard.update();

    // For PidTuningOnly
    if (SmartDashboard.getNumber("Intake P", IntakeConstants.INTAKE_Pivot_P) != intakepivotPIDR.getP()) {
      intakepivotPIDR.setP(SmartDashboard.getNumber("Intake P",
          IntakeConstants.INTAKE_Pivot_P));
    }
    if (SmartDashboard.getNumber("Intake D", IntakeConstants.INTAKE_Pivot_D) != intakepivotPIDR.getD()) {
      intakepivotPIDR.setD(SmartDashboard.getNumber("Intake D",
          IntakeConstants.INTAKE_Pivot_D));
    }
    // // //

    intakepivotPIDR.setFF(FFCalculator.getInstance().calculateIntakeFF());
  }

  // Gets the RPM of the intake motor
  public double getIntakeRpm() {
    return intakeMotor.getEncoder().getVelocity();
  }

  // Gets the Angle of the intake pivot in degrees
  public double getIntakePivotAngle_deg() {
    return intakePivotABSEncoderR.getPosition();
  }

  // Sets the intake pivot FF values
  public Command setIntakeFFValue(double IntakeFFValue) {
    return this.runOnce(() -> intakepivotPIDR.setFF(IntakeFFValue));
  }

  // Zeros the Intake Angle
  public void zeroIntakePivot() {
    // intakePivotABSEncoderR.setZeroOffset(intakePivotABSEncoderR.getPosition());
    intakePivotABSEncoderR.setZeroOffset(MathUtil
        .inputModulus(intakePivotABSEncoderR.getPosition() + intakePivotABSEncoderR.getZeroOffset(), 0, 360));  }

  // Sets the intake rpm to a certain value from -1 to 1
  public Command setIntakeRpmRAW(double input) {
    return this.runOnce(() -> intakeMotor.set(input));
  }

  // Sets the intake pivot angle to a certain angle using PID on right motors
  // **set in degrees**
  public Command setIntakePivotAngle(double setpoint_deg) {
    double setpoint_deg_clamped = MathUtil.clamp(setpoint_deg, 2,178);
    return this.run(() -> intakepivotPIDR.setReference(setpoint_deg_clamped, ControlType.kPosition));
  }

  // For Testing
  public Command intakePivotRaw(double input_test) {
    return this.runOnce(() -> intakePivotMotorR.set(input_test));
  }
}