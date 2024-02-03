// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.commands.arm.ElbowDefaultCommand;
import frc.robot.commands.arm.WristDefaultCommand;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  public static boolean ARM_ROTATION_MANUAL_OVERRIDE = false;
  public static boolean WRIST_ROTATION_MANUAL_OVERRIDE = false;

  public static final ElbowDefaultCommand armDefaultCommand = new ElbowDefaultCommand();
  public static final WristDefaultCommand wristDefaultCommand = new WristDefaultCommand();

  public static final ElbowSubsystem ARM_SUBSYSTEM = new ElbowSubsystem();
  public static final WristSubsystem WRIST_SUBSYSTEM = new WristSubsystem();

  public Robot() {}

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //
    WRIST_SUBSYSTEM.setDefaultCommand(wristDefaultCommand);
    ARM_SUBSYSTEM.setDefaultCommand(armDefaultCommand);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
