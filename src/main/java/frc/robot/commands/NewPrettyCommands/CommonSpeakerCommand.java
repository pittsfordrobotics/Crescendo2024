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
public class CommonSpeakerCommand extends SequentialCommandGroup {
    /** Creates a new SpeakerCommand. */
    public CommonSpeakerCommand(Shooter shooter, Intake intake, double ShooterAngle, double ShooterRPM) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new ParallelCommandGroup(intake.spinIntakeCommand(RobotConstants.SUBWOOF_IntakeSpeed),
                        shooter.setShooterRPMCommand(ShooterRPM)),
                new SequentialCommandGroup(
                        intake.setPivotAngleCommand(35),
                        intake.waitForPivotAngleCommand(),
                        shooter.setPivotAngleCommand(ShooterAngle),
                        shooter.waitForPivotAngleCommand(2).withTimeout(1.5)),
                new InstantCommand(
                        () -> StructureStates.setCurrentState(StructureStates.structureState.commonSpeaker)));
    }
}
