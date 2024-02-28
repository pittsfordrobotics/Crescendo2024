// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NewPrettyCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotConstants;
import frc.robot.lib.StructureStates;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StoredCommand extends SequentialCommandGroup {
    /** Creates a new StoredCommand. */
    public StoredCommand(Shooter shooter, Intake intake) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new InstantCommand(() -> StructureStates.setCurrentState(StructureStates.structureState.stored)),
                new ParallelCommandGroup(intake.spinIntakeCommand(RobotConstants.STORED_IntakeSpeed),
                        shooter.setShooterRPMCommand(RobotConstants.STORED_ShooterRPM)),
                new SequentialCommandGroup(
                        shooter.setPivotAngleCommand(RobotConstants.STORED_ShooterPivotAngle),
                        intake.setPivotAngleCommand(RobotConstants.STORED_IntakePivotAngle))
                        );
    }
}
