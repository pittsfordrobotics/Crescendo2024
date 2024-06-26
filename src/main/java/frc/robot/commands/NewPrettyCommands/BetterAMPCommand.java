// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NewPrettyCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.RobotConstants;
import frc.robot.lib.StructureStates;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BetterAMPCommand extends SequentialCommandGroup {
    /** Creates a new AmpCommand. */
    public BetterAMPCommand(Shooter shooter, Intake intake) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new ParallelCommandGroup(
                        intake.spinIntakeCommand(RobotConstants.NEWAMP_IntakeSpeed_STAGE1),
                        shooter.setShooterRPMCommand(RobotConstants.NEWAMP_ShooterRPM_STAGE1)),
                new SequentialCommandGroup(
                        shooter.setPivotAngleCommand(
                                RobotConstants.NEWAMP_ShooterPivotAngle),
                        intake.setPivotAngleCommand(
                                RobotConstants.NEWAMP_IntakePivotAngle_STAGE1),
                        intake.waitForPivotAngleCommand()),
                new WaitCommand(.5),
                shooter.spinIndexerCommand(RobotConstants.INDEXER_SHOOT_SPEED),
                new WaitCommand(RobotConstants.NEWAMP_WaitTime_STAGE1),
                intake.spinIntakeCommand(RobotConstants.NEWAMP_IntakeSpeed_STAGE2),
                shooter.spinIndexerCommand(RobotConstants.INDEXER_IDLE_SPEED),
                intake.setPivotAngleCommand(
                        RobotConstants.NEWAMP_IntakePivotAngle_STAGE2),
                intake.waitForPivotAngleCommand(5),
                new InstantCommand(() -> StructureStates
                        .setCurrentState(StructureStates.structureState.amp)));
    }
}
