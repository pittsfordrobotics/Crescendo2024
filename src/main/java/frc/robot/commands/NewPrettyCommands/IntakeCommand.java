// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NewPrettyCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.RobotConstants;
import frc.robot.lib.StructureStates;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class IntakeCommand extends SequentialCommandGroup {
  /** Creates a new StartIntake2. */
  public IntakeCommand(Shooter shooter, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
        new ParallelCommandGroup(shooter.setShooterPivotangle(RobotConstants.INTAKE_ShooterPivotAngle),
            intake.setIntakePivotAngle(RobotConstants.INTAKE_IntakePivotAngle)),
        new ParallelCommandGroup(shooter.setshooterRPM(RobotConstants.INTAKE_Shooter_Speed),
            intake.setIntakeRpmRAW(RobotConstants.INTAKE_Intake_Speed)),
        new WaitUntilCommand(() -> shooter.getLimitSwitch()),
        new StoredCommand(shooter, intake));
    StructureStates.setCurrentState(StructureStates.structureState.intake);
  }
}