// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.shootersubsystem;

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
  public static final CommandXboxController COMMAND_DRIVE_CONTROLLER = new CommandXboxController(
      RobotMap.DRIVE_CONTROLLER_PORT);
  public static final XboxController DRIVE_CONTROLLER = COMMAND_DRIVE_CONTROLLER.getHID();
public static final shootersubsystem SHOOTERSUBSYSTEM = new shootersubsystem();


  private DigitalInput frontsensor = new DigitalInput(3);
  private DigitalInput middlesensor = new DigitalInput(1);
  private DigitalInput backsensor = new DigitalInput(2);

  @Override
  public void robotPeriodic() {
/* 
    boolean rightbumper = DRIVE_CONTROLLER.getRightBumper();
    if (rightbumper) {
      _leftShooterMotor.set(.2);
      _rightShooterMotor.set(.2);
    } else {
      _leftShooterMotor.set(0);
      _rightShooterMotor.set(0);
    }

    boolean leftbumper = DRIVE_CONTROLLER.getLeftBumper();

    boolean a = DRIVE_CONTROLLER.getAButton();
    if (a) {
      _intakemotorFx.set(-.1);
      _intakemotorFx2.set(-.1);
    } else if (leftbumper) {
      _intakemotorFx.set(.25);
      _intakemotorFx2.set(.25);

    } else {
      _intakemotorFx.set(0);
      _intakemotorFx2.set(0);
    }

    var frontsensorcsp = !frontsensor.get();
    SmartDashboard.putBoolean("frontsensorcsp", frontsensorcsp);

    var middlesensorcsp = !middlesensor.get();
    SmartDashboard.putBoolean("middlesensorcsp", middlesensorcsp);

    var backsensorcsp = !backsensor.get();
    SmartDashboard.putBoolean("backsensorcsp", backsensorcsp);

    var _leftShooterMotorcsp = _leftShooterMotor.getVelocity().getValueAsDouble();
    SmartDashboard.putNumber("_leftShooterMotorcsp", _leftShooterMotorcsp);

    var _rightShooterMotorcsp = _rightShooterMotor.getVelocity().getValueAsDouble();
    SmartDashboard.putNumber("_rightShooterMotorcsp", _rightShooterMotorcsp);

    boolean x = DRIVE_CONTROLLER.getXButton();
    if (x) {
      _intakemotorFx.set(.5);
      _intakemotorFx2.set(.5);
    }
    if (!a && !x) {

      if (frontsensorcsp) {
        _intakemotorFx.set(.25);
        _intakemotorFx2.set(.25);
      } else if (backsensorcsp) {
        _intakemotorFx.set(-.05);
        _intakemotorFx2.set(-.05);
      }
    }*/

    CommandScheduler.getInstance().run();
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    COMMAND_DRIVE_CONTROLLER.rightBumper().whileTrue(new ShootCommand());
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
    System.out.println("disabled init");
  }

  @Override
  public void disabledPeriodic() {
  }
}
