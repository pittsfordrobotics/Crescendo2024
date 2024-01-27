// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DisabledInstantCommand;
import frc.robot.commands.SwerveDriveXbox;
import frc.robot.commands.SwerveDriveXboxRobotOriented;
import frc.robot.commands.ZeroGyro;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Swerve m_swerveDrive = new Swerve();
  private final Climber CLIMBER = new Climber();
  //private final Shooter m_shooter = new Shooter();
  SimpleWidget fieldOrientedButton;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Shuffleboard.getTab("CONFIG").add(m_swerveDrive);
    fieldOrientedButton = Shuffleboard.getTab("CONFIG").add("Field Oriented Steering", false);
    Shuffleboard.getTab("CONFIG").add("Update Swerve Settings", new DisabledInstantCommand(() -> {
      if(m_swerveDrive.getCurrentCommand() != null){
        m_swerveDrive.getCurrentCommand().cancel();
      }
      m_swerveDrive.setDefaultCommand(isFieldOriented() ? new SwerveDriveXbox(m_swerveDrive) : new SwerveDriveXboxRobotOriented(m_swerveDrive));
    }));
    m_swerveDrive.setDefaultCommand(isFieldOriented() ? new SwerveDriveXbox(m_swerveDrive) : new SwerveDriveXboxRobotOriented(m_swerveDrive)); //Initialization of default command
    // Configure the trigger bindings
    configureBindings();
  }

  private boolean isFieldOriented() {
    return fieldOrientedButton.getEntry().getBoolean(false);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Calls the command ZeroGyro when the right bumper on the drivers controller is pressed
    ZeroGyro zeroGyro = new ZeroGyro(m_swerveDrive);
    m_driverController.rightBumper().whileTrue(zeroGyro);
    //DriveShooter shooterCommand = new DriveShooter(m_shooter, m_driverController::getRightTriggerAxis, m_driverController::getLeftTriggerAxis);
    //m_shooter.setDefaultCommand(shooterCommand);
    m_driverController.x().toggleOnTrue(CLIMBER.runOnce(() -> {
      CLIMBER.extend();
    }));
    m_driverController.x().toggleOnFalse(CLIMBER.runOnce(() -> {
      CLIMBER.retract();
    }));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Choreo swerve auto
    return Autos.choreoSwerveAuto(m_swerveDrive, "NewPath");
  }
}
