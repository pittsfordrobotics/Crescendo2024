package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;

    private SparkPIDController leftPID;
    private SparkPIDController rightPID;
    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;
    private boolean leftZeroReached; // Eventually use the limit switch to determine this
    private boolean rightZeroReached; // Eventually use the limit switch to determine this

    public Climber() {
        // Initialize the left motor.
        leftMotor = new CANSparkMax(ClimberConstants.CAN_CLIMBER_L, MotorType.kBrushless);
        leftMotor.restoreFactoryDefaults();
        leftMotor.setSmartCurrentLimit(20);
        leftMotor.burnFlash();

        // Initialize the right motor.
        rightMotor = new CANSparkMax(ClimberConstants.CAN_CLIMBER_R, MotorType.kBrushless);
        rightMotor.restoreFactoryDefaults();
        rightMotor.setSmartCurrentLimit(20);
        rightMotor.burnFlash();

        // PID Setup
        rightPID = rightMotor.getPIDController();
        rightEncoder = rightMotor.getEncoder();
        rightPID.setFeedbackDevice(rightMotor.getEncoder());
        rightPID.setP(ClimberConstants.CAN_CLIMBER_P);
        rightPID.setI(ClimberConstants.CAN_CLIMBER_I);
        rightPID.setD(ClimberConstants.CAN_CLIMBER_D);

        leftPID = leftMotor.getPIDController();
        leftEncoder = leftMotor.getEncoder();
        leftPID.setFeedbackDevice(leftMotor.getEncoder());
        leftPID.setP(ClimberConstants.CAN_CLIMBER_P);
        leftPID.setI(ClimberConstants.CAN_CLIMBER_I);
        leftPID.setD(ClimberConstants.CAN_CLIMBER_D);

        // // For PidTuningOnly
        // SmartDashboard.putNumber("Climber P", climberPID.getP());
        // SmartDashboard.putNumber("Climber D", climberPID.getD());
        // // // //

        // Zero encoder (assumed zero at startup)
        rightEncoder.setPosition(0);
        leftEncoder.setPosition(0);
    }

    /**
     * Display & update motor status
     */
    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Motor Rotations",
        // rightMotor.getEncoder().getPosition());

        // // For PidTuningOnly
        // if (SmartDashboard.getNumber("Climber P", ClimberConstants.CAN_CLIMBER_P) !=
        // climberPID.getP()) {
        // climberPID.setP(SmartDashboard.getNumber("Climber P",
        // ClimberConstants.CAN_CLIMBER_P));
        // }
        // if (SmartDashboard.getNumber("Climber D", ClimberConstants.CAN_CLIMBER_D) !=
        // climberPID.getD()) {
        // climberPID.setD(SmartDashboard.getNumber("Climber D",
        // ClimberConstants.CAN_CLIMBER_D));
        // }
        // // // //
    }

    // Use pid to reach a predetermined height (Rotations)
    public Command extendCommand() {
        return this.runOnce(() -> {
            rightPID.setReference(ClimberConstants.EXTENSION_SETPOINT, ControlType.kPosition);
            leftPID.setReference(ClimberConstants.EXTENSION_SETPOINT, ControlType.kPosition);
        });
    }

    // Use pid to retract to a predetermined height (Rotations)
    public Command retractCommand() {
        return this.runOnce(() -> {
            rightPID.setReference(ClimberConstants.RETRACTION_SETPOINT, ControlType.kPosition);
            leftPID.setReference(ClimberConstants.RETRACTION_SETPOINT, ControlType.kPosition);
        });
    }

    // Make the climber motor rotate to zero
    public Command moveToZeroCommand() {
        return this.runOnce(() -> rightPID.setReference(0, ControlType.kPosition));
    }

    /**
     * @return a command to zero encoder.
     */
    public Command zeroEncoderCommand() {
        return this.runOnce(() -> rightEncoder.setPosition(0));
    }

    /**
     * <h3>TEMPORARY</h3>
     * <p>
     * Should be removed when limit switches are added.
     * </p>
     * 
     * @return
     *         <p>
     *         A command that drives motors down at 10% power and max 3 amps of
     *         current.
     *         </p>
     *         <p>
     *         Sets zero points for encoders once reached.
     *         </p>
     */
    public Command moveAndZeroEncoderCommand() {
        return this.run(() -> {
            if (leftMotor.getOutputCurrent() < 3) {
                leftMotor.set(-0.1);
                leftZeroReached = false;
            } else {
                leftMotor.set(0);
                leftEncoder.setPosition(0);
                leftZeroReached = true;
            }
            if (rightMotor.getOutputCurrent() < 3) {
                rightMotor.set(-0.1);
                rightZeroReached = false;
            } else {
                rightMotor.set(0);
                rightEncoder.setPosition(0);
                rightZeroReached = true;
            }
        }).until(() -> leftZeroReached && rightZeroReached).beforeStarting(() -> {
            rightZeroReached = false;
            leftZeroReached = false;
        });
    }

    // set voltage for both motors command
    public Command setSpeedCommand(double speed) {
        return this.runOnce(() -> {
            rightMotor.set(speed);
            leftMotor.set(speed);
        });
    }
}
