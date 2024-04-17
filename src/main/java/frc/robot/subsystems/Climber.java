package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.commands.DisabledInstantCommand;

import java.util.ArrayList;
import java.util.Arrays;

/* Process to instal soft limits
Find Limits:
        1. Comment out soft limits setting
        2. Run CLimber down with the .2 button (right trigger on operator)
        3. Zero Encoder
        4. Run climber up w/ .2 button
        5. Record the encoder values when at the top (well call it X for now) (170)
Find Drift:
        1. In climber conmstants set the fwrd limit to X * .8
        2. Set the bkwd limit to X * .2
        3. Run the climber at .5
        4. If the drift isnt over X*.025 run at .7
        5. Continue till running at 1 or the drift is X*.025. 
        7. If at 1 speed lower the ramp rate till an aceptable level (>.25)
                ** Keeping drift below X*.025**
        6. Redord max drift at 1 speed (call it D) (2)
Final Test:
        1. Set the soft limits to X -3D and 0 + 3D (+- 6)
        2. Set the speed bound to right trigger to 1
        3. Run and pray it doesnt break
        4. commit
*/
public class Climber extends SubsystemBase {

        private CANSparkMax leftMotor;
        private CANSparkMax rightMotor;

        private RelativeEncoder leftEncoder;
        private RelativeEncoder rightEncoder;
        private ArrayList<Double> recentCurrentsLeft = new ArrayList<>();
        private ArrayList<Double> recentCurrentsRight = new ArrayList<>();
        private double currentAverageLeft;
        private double currentAverageRight;

        public Climber() {
                // Initialize the left motor.
                leftMotor = new CANSparkMax(ClimberConstants.CAN_CLIMBER_L, MotorType.kBrushless);
                leftMotor.restoreFactoryDefaults();
                leftMotor.setSmartCurrentLimit(40);
                leftMotor.setOpenLoopRampRate(0.5);
                leftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
                leftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
                leftMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward,
                                (float) ClimberConstants.SOFT_LIMIT_FORWARD_LEFT);
                leftMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse,
                                (float) ClimberConstants.SOFT_LIMIT_REVERSE_LEFT);
                leftMotor.burnFlash();

                // Initialize the right motor.
                rightMotor = new CANSparkMax(ClimberConstants.CAN_CLIMBER_R, MotorType.kBrushless);
                rightMotor.restoreFactoryDefaults();
                rightMotor.setSmartCurrentLimit(40);
                rightMotor.setOpenLoopRampRate(0.5);
                rightMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
                rightMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
                rightMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward,
                                (float) ClimberConstants.SOFT_LIMIT_FORWARD_RIGHT);
                rightMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse,
                                (float) ClimberConstants.SOFT_LIMIT_REVERSE_RIGHT);
                rightMotor.burnFlash();

                rightEncoder = rightMotor.getEncoder();
                leftEncoder = leftMotor.getEncoder();

                zeroEncoder();


                // set buttons on shuffleboard

                // Shuffleboard.getTab("Climber").addDouble("Left Motor Current", () -> currentAverageLeft);
                // Shuffleboard.getTab("Climber").addDouble("Right Motor Current", () -> currentAverageRight);

                Shuffleboard.getTab("Climber").addDouble("Left Motor Encoder", () -> leftEncoder.getPosition());
                Shuffleboard.getTab("Climber").addDouble("Right Motor Encoder", () -> rightEncoder.getPosition());

                Shuffleboard.getTab("Climber").add("Zero Encoder", new DisabledInstantCommand(this::zeroEncoder));

                Shuffleboard.getTab("Climber").add("Climber UP", new InstantCommand(() -> setSpeed(.2)));
                Shuffleboard.getTab("Climber").add("Climber DOWN", new InstantCommand(() -> setSpeed(-0.2)));
                Shuffleboard.getTab("Climber").add("Climber STOP", new InstantCommand(() -> setSpeed(0)));
        }

        /**
         * Display & update motor status
         */
        @Override
        public void periodic() {
                // moving average of the current applied to the motors
                // recentCurrentsLeft.add(leftMotor.getOutputCurrent());
                // if (recentCurrentsLeft.size() > 5)
                //         recentCurrentsLeft.remove(0);
                // currentAverageLeft = recentCurrentsLeft.stream()
                //                 .mapToDouble(a -> a)
                //                 .average()
                //                 .orElse(0); // averages all elements in recentCurrents

                // recentCurrentsRight.add(rightMotor.getOutputCurrent());
                // if (recentCurrentsRight.size() > 5)
                //         recentCurrentsRight.remove(0);
                // currentAverageRight = recentCurrentsRight.stream()
                //                 .mapToDouble(a -> a)
                //                 .average()
                //                 .orElse(0);
        }

        public void zeroEncoder() {
                leftEncoder.setPosition(0);
                rightEncoder.setPosition(0);
        }

        /**
         * @return a command to zero the encoder.
         * 
         */
        public Command zeroEncoderCommand() {
                return this.runOnce(() -> {
                        zeroEncoder();
                });
        }


        // Set the speed of the climber motors
        public void setSpeed(double speed) {
                leftMotor.set(speed);
                rightMotor.set(speed);
        }

        /**
         * @return a command that sets the voltage for both motors.
         */
        public Command setSpeedCommand(double speed) {
                return this.runOnce(() -> {
                        setSpeed(speed);
                });
        }

        /**
         * Detects a chain, based on the principle that if we're on a chain the average
         * current should be > usual
         * 
         * @return a new boolean: {left chain, right chain}
         */
        public boolean[] detectChain() {
                return new boolean[] {
                                currentAverageLeft > ClimberConstants.CLIMBER_CURRENT_THRESHOLD,
                                currentAverageRight > ClimberConstants.CLIMBER_CURRENT_THRESHOLD
                };
        }

        /**
         * @return a command that moves both motors down until one hits the chain, then
         *         stops that one and moves the
         *         other until both are on the chain.
         */
        public Command downUntilChainCommand() {
                return this.run(() -> {
                        boolean[] chain = detectChain();
                        leftMotor.set(chain[0] ? 0 : -0.5);
                        rightMotor.set(chain[1] ? 0 : -0.5);
                }).until(() -> Arrays.equals(detectChain(), new boolean[] { true, true }));
        }
}
