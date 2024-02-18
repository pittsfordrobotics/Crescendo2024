// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NewPrettyCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotConstants;
import frc.robot.lib.StructureStates;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpCommand extends SequentialCommandGroup {
  /** Creates a new AmpCommand. */
  public AmpCommand(Shooter shooter, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(intake.spinIntakeCommand(RobotConstants.AMP_IntakeSpeed),
            shooter.setShooterRPMCommand(RobotConstants.AMP_ShooterRPM)),
        new SequentialCommandGroup(shooter.setShooterPivotangle(RobotConstants.AMP_ShooterPivotAngle),
            intake.setPivotAngleCommand(RobotConstants.AMP_IntakePivotAngle)));
    StructureStates.setCurrentState(StructureStates.structureState.amp);
  }
}
