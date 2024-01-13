// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.commands.EndEffectorIdle;

public class EndEffector extends SubsystemBase {
  private static final int numCycles = 15;
  private CANSparkMax leftMotor = EndEffectorConstants.leftMotor;
  private CANSparkMax rightMotor = EndEffectorConstants.rightMotor;
  private ArrayList<Double> lastCurrent = new ArrayList<>(numCycles);
  /** Creates a new EndEffector. */
  public EndEffector() {
    leftMotor.restoreFactoryDefaults();
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.restoreFactoryDefaults();
    rightMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.follow(leftMotor, true);
    this.setDefaultCommand(new EndEffectorIdle(this));
  }

  public void start(boolean isIdle) {
    if(isIdle) {leftMotor.set(-0.15);}
    else{leftMotor.set(-0.3);}
  }
  public void stop() {
    leftMotor.stopMotor();
  }
  public boolean hasIntaken() {
    if(computeAverage() >= 25) {
      return true;
    }
    else{
      return false;
    }
  }

  private double computeAverage() {
    double sum = 0;
    for(Double value: lastCurrent) {
      sum += value;    
  }
  return sum/numCycles;
}
  @Override
  public void periodic() {
    SmartDashboard.putNumber("left_motor_current", leftMotor.getOutputCurrent());
    SmartDashboard.putNumber("left_motor_voltage", leftMotor.getAppliedOutput());

    lastCurrent.add(leftMotor.getOutputCurrent());
    if(lastCurrent.size() > numCycles) {
      lastCurrent.remove(0);
    }
    SmartDashboard.putNumber("rolling average current", computeAverage());
    // This method will be called once per scheduler run
  }
}
