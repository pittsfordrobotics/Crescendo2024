package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;


public class DriveSwerve extends Command {
    private SwerveSubsystem swerveDrive;
    private CommandXboxController controller;

    public DriveSwerve(SwerveSubsystem swerveDrive) {
        this.swerveDrive = swerveDrive;
        controller = new CommandXboxController(OperatorConstants.kDriverControllerPort);
        addRequirements(swerveDrive);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {

    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        double xAxis = MathUtil.applyDeadband(-controller.getLeftX(), SwerveConstants.driverControllerLeftDeadband);
        double yAxis = MathUtil.applyDeadband(-controller.getLeftY(), SwerveConstants.driverControllerLeftDeadband);
        double rotateX = -controller.getRightY();
        double rotateY = -controller.getRightX();
        double radius = MathUtil.applyDeadband(Math.sqrt(Math.pow(rotateX, 2) + Math.pow(rotateY, 2)),
                SwerveConstants.driverControllerRightDeadband);

//        swerveDrive.drive(new Translation2d(xAxis, yAxis), Math.atan2(rotateY, rotateX), true);
        swerveDrive.drive(new Translation2d(xAxis, yAxis), 0, true);
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {

    }
}
