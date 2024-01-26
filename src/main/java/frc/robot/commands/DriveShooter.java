// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class DriveShooter extends Command {
  private Shooter shooter;
  private DoubleSupplier inputSpeed1;
  private DoubleSupplier inputSpeed2;
  /** Creates a new DriveShooter. */
  public DriveShooter(Shooter shooter, DoubleSupplier inputSpeed1, DoubleSupplier inputSpeed2) {
    addRequirements(shooter);
    this.shooter = shooter;
    this.inputSpeed1 = inputSpeed1;
    this.inputSpeed2 = inputSpeed2;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(inputSpeed1.getAsDouble() == 0) {
      shooter.driveShooter(-inputSpeed2.getAsDouble());
    } else {
      shooter.driveShooter(inputSpeed1.getAsDouble());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
