// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;

/** A singular MAXSwerve module. Constains methods for updating inputs and driving the swerve module. */
public class SwerveModuleIO {
    private final CANSparkBase driveMotor;
    private final CANSparkBase steerMotor;
    
    private final RelativeEncoder driveRelativeEncoder;
    private final AbsoluteEncoder steerAbsoluteEncoder;
    private final SparkPIDController drivePID;
    private final SparkPIDController steerPID;
    private final SlewRateLimiter angleRateLimiter = new SlewRateLimiter(SwerveConstants.MODULE_ROTATION_RATE_LIMIT); // Limit the maximum turning rate of swerve module

    // Inputs
    public double drivePositionMeters = 0.0;
    public double driveVelocityMetersPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;
    public double driveTempCelsius = 0.0;

    public double steerAbsolutePositionRad = 0.0;
    public double steerAbsoluteVelocityRadPerSec = 0.0;
    public double steerAppliedVolts = 0.0;
    public double steerCurrentAmps = 0.0;
    public double steerTempCelsius = 0.0;

    /**
     * Constructs a SwerveModuleIO object, which sets up 2 motors and PID controllers when constructed.
     * @param driveID The CAN ID of the drive motor
     * @param steerID The CAN ID of the steering motor
     * @param offset The measured offset of the swerve module
     */
    public SwerveModuleIO(int driveID, int steerID, Rotation2d offset) {
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        driveMotor.clearFaults();
        driveMotor.restoreFactoryDefaults();
        driveMotor.setSmartCurrentLimit(30);
        driveMotor.setSecondaryCurrentLimit(40);
        setDriveBrakeMode(true);

        steerMotor = new CANSparkMax(steerID, MotorType.kBrushless);
        steerMotor.clearFaults();
        steerMotor.restoreFactoryDefaults();
        steerMotor.setSmartCurrentLimit(15);
        steerMotor.setSecondaryCurrentLimit(20);
        setSteerBrakeMode(true);

        driveRelativeEncoder = driveMotor.getEncoder();
        steerAbsoluteEncoder = steerMotor.getAbsoluteEncoder(Type.kDutyCycle);
        steerAbsoluteEncoder.setInverted(true);

        // allows for faster response time
        driveRelativeEncoder.setAverageDepth(2);
        driveRelativeEncoder.setMeasurementPeriod(10);

        // converts to m/s
        driveRelativeEncoder.setPositionConversionFactor((Math.PI * SwerveConstants.WHEEL_DIAMETER_METERS) / SwerveConstants.DRIVE_GEAR_RATIO);
        driveRelativeEncoder.setVelocityConversionFactor(((Math.PI * SwerveConstants.WHEEL_DIAMETER_METERS) / SwerveConstants.DRIVE_GEAR_RATIO) / 60.0);

        // converts to rad/s
        steerAbsoluteEncoder.setPositionConversionFactor(2*Math.PI);
        steerAbsoluteEncoder.setVelocityConversionFactor(2*Math.PI / 60);
        double offsetModulusRad = MathUtil.inputModulus(offset.getRadians(), 0, 2*Math.PI);
        
        System.out.println("SwerveModuleIO: Setting absolute encoder offset of " + steerID + " to " + (offsetModulusRad * 180 / Math.PI) + " degrees.");
        REVLibError result = steerAbsoluteEncoder.setZeroOffset(offsetModulusRad);
        System.out.println("Offset result: " + result.value);
        
        drivePID = driveMotor.getPIDController();
        steerPID = steerMotor.getPIDController();

        drivePID.setFeedbackDevice(driveRelativeEncoder);
        steerPID.setFeedbackDevice(steerAbsoluteEncoder);
        steerPID.setPositionPIDWrappingEnabled(true);
        steerPID.setPositionPIDWrappingMinInput(0);
        steerPID.setPositionPIDWrappingMaxInput(2*Math.PI);
        steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20); // trying to get more position readings

        drivePID.setP(SwerveConstants.MODULE_DRIVE_P);
        drivePID.setI(SwerveConstants.MODULE_DRIVE_I);
        drivePID.setD(SwerveConstants.MODULE_DRIVE_D);
        drivePID.setFF(SwerveConstants.MODULE_DRIVE_FF);

        steerPID.setP(SwerveConstants.MODULE_STEER_P);
        steerPID.setI(SwerveConstants.MODULE_STEER_I);
        steerPID.setD(SwerveConstants.MODULE_STEER_D);

        driveMotor.burnFlash(); // Save the settings to the motor controllers
        steerMotor.burnFlash();
    }
    /** Reads new input data */
    public void updateInputs() {
        //        driveTuner.setPID();
        //        steerTuner.setPID();
        drivePositionMeters = driveRelativeEncoder.getPosition();
        driveVelocityMetersPerSec = driveRelativeEncoder.getVelocity();
        driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        driveCurrentAmps = driveMotor.getOutputCurrent();
        driveTempCelsius = driveMotor.getMotorTemperature();
        
        steerAbsolutePositionRad = steerAbsoluteEncoder.getPosition();
        steerAbsoluteVelocityRadPerSec = steerAbsoluteEncoder.getVelocity();
        steerAppliedVolts = steerMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        steerCurrentAmps = steerMotor.getOutputCurrent();
        steerTempCelsius = steerMotor.getMotorTemperature();
    }
    /**
     * Updates inputs and returns the current swerve module steering angle
     * @return steering angle of the swerve module, in degrees from 0-360
     */
    public double getCurrentAngleDeg() {
        updateInputs();
        return MathUtil.inputModulus(Math.toDegrees(steerAbsolutePositionRad), 0, 360);
    }
    /** <p>Drives the swerve drive based on desired swerve module state.</p>
     * <p>If using closed loop PID, set isOpenLoop to false.</p>
     * @param targetState Target swerve module state.
     * @param isOpenLoop Set to false to enable closed loop PID.
     */
    public void drive(SwerveModuleState targetState, boolean isOpenLoop) {
        if(isOpenLoop) { // Target drive motor volts from desired speed, max voltage, max speed.
            double driveVolts = targetState.speedMetersPerSecond / SwerveConstants.MAX_LINEAR_VELOCITY_METERS_PER_SECOND * 12;
            double inputVolts = driveMotor.getBusVoltage();
            if(driveVolts > inputVolts){
                driveVolts = inputVolts;
            }
            driveMotor.setVoltage(driveVolts);
        }
        else { // use closed loop PID to determine drive speed and steer position
            drivePID.setReference(targetState.speedMetersPerSecond, ControlType.kVelocity);
            steerPID.setReference(angleRateLimiter.calculate(targetState.angle.getRadians()), ControlType.kPosition, 0); // Set steering angle, with slew rate limiter.
        }
    }
    public void setDriveBrakeMode(boolean enable) {
        driveMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }
    public void setSteerBrakeMode(boolean enable) {
        steerMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }
    public void stopMotors() {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }
    public void setZeroOffset(double offset) {
        System.out.println("SetZeroOffset: setting offset of " + steerMotor.getDeviceId() + " to " + (offset * 180 / Math.PI) + " degrees.");
        REVLibError result = steerAbsoluteEncoder.setZeroOffset(offset);
        System.out.println("SetOffset result: " + result.value);
    }
}
