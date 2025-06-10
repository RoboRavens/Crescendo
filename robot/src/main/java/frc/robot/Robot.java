// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the manifest
 * file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  public static final CommandXboxController COMMAND_DRIVE_CONTROLLER = new CommandXboxController(RobotMap.DRIVE_CONTROLLER_PORT);
  public static final XboxController DRIVE_CONTROLLER = COMMAND_DRIVE_CONTROLLER.getHID();

  private TalonFX _shooterMotorLeft = new TalonFX(RobotMap.SHOOTER_LEFT_MOTOR_CAN_ID);
  private TalonFX _shooterMotorRight = new TalonFX(RobotMap.SHOOTER_RIGHT_MOTOR_CAN_ID);
  private TalonFX _intakeMotorTop = new TalonFX(RobotMap.INTAKE_MOTOR_TOP_CAN_ID);
  private TalonFX _intakeMotorBottom = new TalonFX(RobotMap.INTAKE_MOTOR_BOTTOM_CAN_ID);


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    Command m_autonomousCommand = new PrintCommand("auto mode");

    // // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {

  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }
}
