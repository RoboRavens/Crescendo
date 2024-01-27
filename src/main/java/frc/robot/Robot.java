// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.LEDsSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  public XboxController Xbox = new XboxController(0);

  public LEDsSubsystem ledsSusystem = new LEDsSubsystem();
  public Robot() {}

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

  }

  @Override
  public void robotPeriodic(){
    boolean aButton = Xbox.getAButton();

    if (aButton) {
      System.out.println("a button pressed");
      ledsSusystem.setColor(0, 255, 0);
    }

    boolean bButton = Xbox.getBButton();
    if (bButton) {
      ledsSusystem.setColor(255, 0, 0);
    }

    boolean xButton = Xbox.getXButton();
    if (xButton) {
      ledsSusystem.setColor(0, 0, 255);
    }

    boolean yButton = Xbox.getYButton();
    if (yButton) {
      ledsSusystem.setColor(127, 127, 0);
    }
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
