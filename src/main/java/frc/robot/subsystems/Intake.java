// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.DisabledInstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private CANSparkMax intakeMotor;
  private CANSparkMax intakePivotMotorL;
  private CANSparkMax intakePivotMotorR;

  private SparkPIDController intakepivotPIDR;

  private SparkAbsoluteEncoder intakePivotEncoderR;

  /** Creates a new Intake. */
  public Intake() {
    // Intake Pivot Motor L
    intakePivotMotorL = new CANSparkMax(IntakeConstants.CAN_INTAKE_PIVOT_L, MotorType.kBrushless);
    intakePivotMotorL.restoreFactoryDefaults();
    intakePivotMotorL.burnFlash();
    intakePivotMotorL.setSmartCurrentLimit(20);
    intakePivotMotorL.follow(intakePivotMotorR, false);

    // Intake Pivot Motor R (Leader)
    intakePivotMotorR = new CANSparkMax(IntakeConstants.CAN_INTAKE_PIVOT_R, MotorType.kBrushless);
    intakePivotMotorR.restoreFactoryDefaults();
    intakePivotMotorR.burnFlash();
    intakePivotMotorR.setSmartCurrentLimit(20);

    // Intake Motor
    intakeMotor = new CANSparkMax(IntakeConstants.CAN_INTAKE, MotorType.kBrushless);
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.burnFlash();
    intakeMotor.setSmartCurrentLimit(20);

    // Intake Pivot Pid (in the right motor controller)
    intakepivotPIDR = intakePivotMotorR.getPIDController();
    intakepivotPIDR.setFeedbackDevice(intakePivotEncoderR);
    intakepivotPIDR.setP(IntakeConstants.INTAKE_Pivot_P);
    intakepivotPIDR.setI(IntakeConstants.INTAKE_Pivot_I);
    intakepivotPIDR.setD(IntakeConstants.INTAKE_Pivot_D);

    // Puts a button on the shuffleboard to zero the intake pivot
    Shuffleboard.getTab("Intake").add("Zero Intake Pivot", new DisabledInstantCommand(this::zeroIntakePivot));
  }

  @Override
  public void periodic() {
    Shuffleboard.getTab("Intake").add("Intake RPM", this.getIntakeRpm());
    Shuffleboard.getTab("Intake").add("Intake Pivot Angle", this.getIntakePivotAngle());
  }

  // Gets the RPM of the intake motor
  public double getIntakeRpm() {
    return intakeMotor.getEncoder().getVelocity();
  }

  // Gets the Angle of the intake pivot
  public double getIntakePivotAngle() {
    return intakePivotEncoderR.getPosition();
  }

  // Zeros the Intake Angle
  public void zeroIntakePivot() {
    intakePivotEncoderR.setZeroOffset(intakePivotEncoderR.getPosition());
  }

  // Sets the intake rpm to a certain value from -1 to 1
  public void setIntakeRpm(double input) {
    intakeMotor.set(input);
  }

  // Sets the intake pivot angle to a certain angle using PID on right motor
  public Command setIntakePivotAngle(double setpoint) {
    return this.run(() -> intakepivotPIDR.setReference(setpoint, ControlType.kVelocity));
  }

  // For Testing
  public Command intakePivotRaw(double input) {
    return this.runOnce(() -> intakePivotMotorR.set(input));
  }
}