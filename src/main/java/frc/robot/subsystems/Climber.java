package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

import java.util.ArrayList;

public class Climber extends SubsystemBase {

    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;
    private ArrayList<Double> recentCurrentsLeft = new ArrayList<>();
    private ArrayList<Double> recentCurrentsRight = new ArrayList<>();
    private double currentAverageLeft;
    private double currentAverageRight;
    private double softLimit;

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

        rightEncoder = rightMotor.getEncoder();
        leftEncoder = leftMotor.getEncoder();

        // Zero encoder (assumed zero at startup)
        rightEncoder.setPosition(0);
        leftEncoder.setPosition(0);

        //soft limits
        softLimit = leftEncoder.getPosition() * 0.5;
        leftMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) softLimit);
        rightMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) softLimit);

        //set buttons on shuffleboard
        Shuffleboard.getTab("Climber").addDouble("Soft Limit", () -> softLimit);
        //todo: name the buttons (how?)
        Shuffleboard.getTab("Climber").add(sendableBuilder -> softLimit *= 1.1); //up 10%
        Shuffleboard.getTab("Climber").add(sendableBuilder -> softLimit *= 0.9); //down 10%
    }

    /**
     * Display & update motor status
     */
    @Override
    public void periodic() {
        //moving average of the current applied to the motors
        recentCurrentsLeft.add(leftMotor.getOutputCurrent());
        if(recentCurrentsLeft.size() > 5) recentCurrentsLeft.remove(0);
        currentAverageLeft = recentCurrentsLeft.stream()
                .mapToDouble(a -> a)
                .average()
                .orElse(0); //averages all elements in recentCurrents

        recentCurrentsRight.add(rightMotor.getOutputCurrent());
        if(recentCurrentsRight.size() > 5) recentCurrentsRight.remove(0);
        currentAverageRight = recentCurrentsRight.stream()
                .mapToDouble(a -> a)
                .average()
                .orElse(0);
    }

    /**
     * @return a command to zero the encoder.
     */
    public Command zeroEncoderCommand() {
        return this.runOnce(() -> {
            leftEncoder.setPosition(0);
            rightEncoder.setPosition(0);
        });
    }

    /**
     * @return a command that sets the voltage for both motors.
     */
    public Command setSpeedCommand(double speed) {
        return this.runOnce(() -> {
            rightMotor.set(speed);
            leftMotor.set(speed);
        });
    }

    /**
     * Detects a chain, based on the principle that if we're on a chain the average current should be > usual
     * @return a new boolean: {left chain, right chain}
     */
    public boolean[] detectChain() {
        return new boolean[] {
                currentAverageLeft > ClimberConstants.CLIMBER_OFFSET,
                currentAverageRight > ClimberConstants.CLIMBER_OFFSET
        };
    }

    /**
     * @return a command that moves both motors down until one hits the chain, then stops that one and moves the
     * other until both are on the chain.
     */
    public Command downUntilChainCommand() {
        return this.run(() -> {
            boolean[] chain = detectChain();
            leftMotor.set(chain[0] ? 0 : -0.5);
            rightMotor.set(chain[1] ? 0 : -0.5);
        }).until(() -> {
            boolean[] chain = detectChain();
            return chain[0] && chain[1];
        });
    }
}
