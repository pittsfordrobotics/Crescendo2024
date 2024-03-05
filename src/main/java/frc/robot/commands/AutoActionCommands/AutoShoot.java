package frc.robot.commands.AutoActionCommands;


import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.NewPrettyCommands.CommonSpeakerCommand;
import frc.robot.commands.NewPrettyCommands.StoredCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoShoot extends SequentialCommandGroup {
    public AutoShoot(Shooter shooter, Intake intake, double shooterAngleDeg, double shooterRPM) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new CommonSpeakerCommand(shooter, intake, shooterAngleDeg, shooterRPM), // Also waits for shooter to reach position with 2 deg tolerance
                shooter.waitForShooterRPMCommand().withTimeout(1.0),
                shooter.spinIndexerCommand(RobotConstants.INDEXER_SHOOT_SPEED),
                Commands.waitSeconds(0.25),
                shooter.spinIndexerCommand(RobotConstants.INDEXER_IDLE_SPEED), // Idle indexer and prepare to intake
                new StoredCommand(shooter, intake)
        );
    }
}