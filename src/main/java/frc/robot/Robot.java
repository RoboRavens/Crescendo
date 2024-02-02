// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LEDsSubsystem;
//import frc.robot.subsystems.LEDsSubsystem2;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  public XboxController Xbox = new XboxController(0);

  public LEDsSubsystem ledsSubsystem = new LEDsSubsystem();
  //public LEDsSubsystem2 ledsSubsystem2 = new LEDsSubsystem2();

  private double angle=0;

  public double getAngle() {
    return angle;
  }


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
    double xAxis = Xbox.getRawAxis(0);
    SmartDashboard.putNumber("xstick", xAxis);

    double yAxis = Xbox.getRawAxis(1);
    SmartDashboard.putNumber("ystick", yAxis);

    double angle180 = Math.atan2(yAxis,xAxis) * (180 / Math.PI);
    SmartDashboard.putNumber("angle180", angle180);

    angle = angle180;
    if(angle180 < 0){
      angle = 365 + angle;
    }
    ledsSubsystem.setInputAngle(angle);

    SmartDashboard.putNumber("angle", angle);



    boolean aButton = Xbox.getAButton();
    if (aButton) {
      System.out.println("a button pressed, green");
      ledsSubsystem.setColor(0, 255, 0);
    }

    boolean bButton = Xbox.getBButton();
    if (bButton) {
      System.out.println("b button pressed, red");
      ledsSubsystem.setColor(255, 0, 0);
    }

    boolean xButton = Xbox.getXButton();
    if (xButton) {
      System.out.println("x button pressed, blue");
      ledsSubsystem.setColor(0, 0, 255);
    }

    boolean yButton = Xbox.getYButton();
    if (yButton) {
      System.out.println("y button pressed, yellow");
      ledsSubsystem.setColor(127, 127, 0);
    }

    boolean rightBumper = Xbox.getRightBumper();
    if (rightBumper) {
      System.out.println("rBumper pressed, no color");
      ledsSubsystem.setColor(0, 0, 0);
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
