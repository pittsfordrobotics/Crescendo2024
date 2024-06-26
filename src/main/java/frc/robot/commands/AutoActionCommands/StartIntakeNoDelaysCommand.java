// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoActionCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotConstants;
import frc.robot.lib.StructureStates;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/**
 * Command group that starts intaking and will run until the intake is in position.
 */
public class StartIntakeNoDelaysCommand extends SequentialCommandGroup {
	/**
	 * Constructs a command that starts intaking and will run until the intake is in position.
	 * @param shooter The instance of the shooter subsystem
	 * @param intake The instance of the intake subsystem
	 */
	public StartIntakeNoDelaysCommand(Shooter shooter, Intake intake) {
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());

		addCommands(
				// Start up intake and shooter speed.
				new ParallelCommandGroup(
          shooter.setShooterRPMCommand(RobotConstants.INTAKE_ShooterRPM),
					intake.spinIntakeCommand(RobotConstants.INTAKE_IntakeSpeed)),
				// Lower shooter and intake and wait for them to drop.
				shooter.setPivotAngleCommand(RobotConstants.INTAKE_ShooterPivotAngle),
				//shooter.waitForPivotAngleCommand(),
				intake.setPivotAngleCommand(RobotConstants.INTAKE_IntakePivotAngle));
				//intake.waitForPivotAngleCommand());

		StructureStates.setCurrentState(StructureStates.structureState.intake);
	}
}