package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimberConstants;

public class Climber extends PIDSubsystem {

    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;

    public Climber() {
        super(new PIDController(ClimberConstants.CAN_CLIMBER_P, ClimberConstants.CAN_CLIMBER_I, ClimberConstants.CAN_CLIMBER_D));
        this.getController().setTolerance(1);
        
        leftMotor = new CANSparkMax(ClimberConstants.CAN_CLIMBER_L, MotorType.kBrushless);
        leftMotor.restoreFactoryDefaults();
        leftMotor.burnFlash();
        leftMotor.setSmartCurrentLimit(20);

        rightMotor = new CANSparkMax(ClimberConstants.CAN_CLIMBER_R, MotorType.kBrushless);
        rightMotor.restoreFactoryDefaults();
        rightMotor.burnFlash();
        rightMotor.setSmartCurrentLimit(20);

        leftMotor.follow(rightMotor);
        rightMotor.getEncoder().setPosition(0);

        this.getController().setSetpoint(0);
        enable();
    }

    public void extend() {
        setSetpoint(ClimberConstants.EXTENSION_SETPOINT);
    }

    public void retract() {
        setSetpoint(0);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        rightMotor.set(this.getController().calculate(output, setpoint));
    }

    @Override
    protected double getMeasurement() {
        SmartDashboard.putNumber("Climber Position (rotations)", rightMotor.getEncoder().getPosition());
        return rightMotor.getEncoder().getPosition();
    }

}

