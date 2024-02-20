// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class AutoCommandFactory {
    // PID controllers
    private SwerveSubsystem swerveSubsystem;
    private PIDController xPIDController;
    private PIDController yPIDController;
    private PIDController headinganglePIDController;
    private Consumer<ChassisSpeeds> speedsConsumer;
    private BooleanSupplier allianceSupplier;
    public AutoCommandFactory(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        // TODO: Replace these values with X and Y translate pid values (should be same) and rotational pid from robot (not necessarily, will test, but should def be nonzero)
        xPIDController = new PIDController(0.002, 0.0, 0.0); // PIDController for field-relative X translation (input: Y error in meters, output: m/s).
        yPIDController = new PIDController(0.002, 0, 0); // PIDController for field-relative Y translation (input: Y error in meters, output: m/s).
        headinganglePIDController = new PIDController(0.6, 0.0, 0.01); // PID controller to correct for rotation error
        headinganglePIDController.enableContinuousInput(-Math.PI, Math.PI); 
        speedsConsumer = (ChassisSpeeds speeds) -> swerveSubsystem.drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), speeds.omegaRadiansPerSecond, false); // Consumes target robot-relative chassis speeds and commands them to the robot
        allianceSupplier = () -> { 
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            return alliance.isPresent() && (alliance.get() == Alliance.Red);
        };
    }
    public Command generateChoreoCommand(ChoreoTrajectory traj) {
        return generateChoreoCommand(traj, 0.0);
    }

    public Command generateChoreoCommand(ChoreoTrajectory traj, double waitSeconds) {
        // return new RunCommand(() -> {System.out.println("AVASDFASDASF\n\n\n\n\n"); swerveSubsystem.drive(new Translation2d(0.1, 0.2), 0.3, false);}, swerveSubsystem);
        return Choreo.choreoSwerveCommand(
            traj,
            swerveSubsystem::getPose,
            xPIDController,
            yPIDController,
            headinganglePIDController,
            speedsConsumer,
            allianceSupplier,
            swerveSubsystem).alongWith(Commands.waitSeconds(waitSeconds));
    }
}
