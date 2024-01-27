package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;

    private SparkPIDController climberPID;

    public Climber() {
        //L Motor
        leftMotor = new CANSparkMax(ClimberConstants.CAN_CLIMBER_L, MotorType.kBrushless);
        leftMotor.restoreFactoryDefaults();
        leftMotor.burnFlash();
        leftMotor.setSmartCurrentLimit(20);

        //R Motor
        rightMotor = new CANSparkMax(ClimberConstants.CAN_CLIMBER_R, MotorType.kBrushless);
        rightMotor.restoreFactoryDefaults();
        rightMotor.burnFlash();
        rightMotor.setSmartCurrentLimit(20);

        leftMotor.follow(rightMotor);
        
        //PID Setup
        climberPID = rightMotor.getPIDController();
        climberPID.setFeedbackDevice(rightMotor.getEncoder());
        climberPID.setP(ClimberConstants.CAN_CLIMBER_P);
        climberPID.setI(ClimberConstants.CAN_CLIMBER_I);
        climberPID.setD(ClimberConstants.CAN_CLIMBER_D);

        //Zero encoder
        rightMotor.getEncoder().setPosition(0);
    }

    //Display & update motor status
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Motor Rotations", rightMotor.getEncoder().getPosition());
    }

    //Use pid to reach a predetermined height (Rotations)
    public Command extend() {
        return this.run(() -> climberPID.setReference(ClimberConstants.EXTENSION_SETPOINT, ControlType.kPosition));
    }

    //Use pid to retract to a predetermined height (Rotations)
    public Command retract() {
        return this.run(() -> climberPID.setReference(ClimberConstants.RETRACTION_SETPOINT, ControlType.kPosition));
    }

    //Make the climber motor rotate to zero
    public Command setToZero() {
        return this.run(() -> climberPID.setReference(0, ControlType.kPosition));
    }

    //Zeros the climber's relative encoder
    public Command zeroEncoder()
    {
        return this.runOnce(() -> rightMotor.getEncoder().setPosition(0));
    }
}

