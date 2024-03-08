// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoActionCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoFireNote extends SequentialCommandGroup {
  /** Creates a new autoSpinShooter. */
  public AutoFireNote(Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      shooter.waitForShooterRPMCommand().withTimeout(1.0),
      shooter.spinIndexerCommand(RobotConstants.INDEXER_SHOOT_SPEED),
      Commands.waitSeconds(0.25),
      shooter.spinIndexerCommand(RobotConstants.INDEXER_IDLE_SPEED)
    );
  }
}
