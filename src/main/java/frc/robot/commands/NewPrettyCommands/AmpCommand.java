// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NewPrettyCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotConstants;
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
        new ParallelCommandGroup(shooter.setShooterPivotangle(RobotConstants.AMP_ShooterPivotAngle),
            intake.setIntakePivotAngle(RobotConstants.AMP_IntakePivotAngle)),
        shooter.setIndexer(RobotConstants.INDEXER_IDLE_SPEED),
        new ParallelCommandGroup(intake.setIntakeRpmRAW(RobotConstants.AMP_IntakeSpeed),
            shooter.setshooterRPM(RobotConstants.AMP_ShooterRPM)));

    // The above sets the position to the amp state but does not release the note
    // I want to add commands that wait for a boolean to be true then set the
    // indexer to AMP_IndexerSpeedRelease
    // Would this work? Passing in a button as the boolean supplier?
    // new WaitUntilCommand(() -> ShouldShoot.getAsBoolean()),
    // shooter.setIndexer(RobotConstants.AMP_IndexerSpeedRelease));
  }
}
