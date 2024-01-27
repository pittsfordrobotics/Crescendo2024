// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private final Shooter SHOOTER = new Shooter();
  private final Intake INTAKE = new Intake();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    // Sets the feedfoeard for both pivots PIDS based on angles
    // ***Look at Diagram for understanding***
    double theta = SHOOTER.getShooterAngle_deg();
    double alpha = INTAKE.getIntakePivotAngle();

    double L1 = ShooterConstants.L1_SpivtoWpivperp;
    double L1CM = ShooterConstants.L1CM1_SpivtoCM1;
    double L2 = IntakeConstants.L2_WpivPerptoWpiv;
    double L3 = IntakeConstants.L3_WpivtoCm2;
    double M1 = ShooterConstants.M1_Total_Mass_of_Shooter;
    double M2 = IntakeConstants.M2_Total_Mass_of_Intake;
    double Theta_CM = theta + ShooterConstants.Theta_Offset;
    double Alpha_CM = alpha + IntakeConstants.Alpha_Offset;

    double CM2X = (L1 * Math.cos(theta)) + (L2 * Math.cos(90 + theta)) + (L3 * Math.cos(Alpha_CM + theta));
    double CM2Y = (L1 * Math.sin(theta)) + (L2 * Math.sin(90 + theta)) + (L3 * Math.sin(Alpha_CM + theta));

    double CM1X = L1CM * Math.cos(Theta_CM);
    double CM1Y = L1CM * Math.sin(Theta_CM);

    double CMX = (CM2X * M2 + CM1X * M1) / (M1 + M2);
    double CMY = (CM2Y * M2 + CM1Y * M1) / (M1 + M2);

    double TotalTorque_ShoulderPiv = (Math.cos(Math.atan(CMY / CMX)) * 9.8 * (M1 + M2))
        * Math.sqrt((CMX * CMX) + (CMY * CMY));
    double TotalTorque_IntakePiv = (Math.cos(theta + Alpha_CM) * 9.8 * (M2)) * L3;

    SHOOTER.setShooterFFvalue (TotalTorque_ShoulderPiv * ShooterConstants.SHOOTER_Pivot_FF_Constant);
    INTAKE.setIntakeFFValue (TotalTorque_IntakePiv * IntakeConstants.INTAKE_Pivot_FF_Constant);


    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
