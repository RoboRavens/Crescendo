// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.LEDsSubsystem2023;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  public XboxController Xbox = new XboxController(0);

  public LEDsSubsystem ledsSubsystem = new LEDsSubsystem();
  //public LEDsSubsystem2023 ledsSubsystem2023 = new LEDsSubsystem2023();
  //public LEDsSubsystem2 ledsSubsystem2 = new LEDsSubsystem2();
  public IntakeSubsystem intakeSubsystem= new IntakeSubsystem();

  private double angle=0;
  int modeNum = 0;
  int firstColor = 1;

  public double getAngle() {
    return angle;
  }


  public Robot() {
  }
  
    
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ledsSubsystem.setColumnHSV(28,1,255,255);
    //ledsSubsystem.setColumnRainbow(1,255,255);
    /*for(int i=1;i<=28;i++)
    {
      if(i%2==0)
      ledsSubsystem.setColumnHSV(i,1,255,255);
    } */
  
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

    /* 
    if(firstColor%5==0){
      ledsSubsystem.setColumnRainbow(firstColor%180,255,255);
      
    }
    firstColor += 1;

    /*for(int i=1;i<=28;i++)
    {
      if(i%2==0)
      //ledsSubsystem.setColumnHSV(i,1,255,255);
      //ledsSubsystem.setColumnHSV(i,0,0,0);
      
      
    }

    
      

  
    boolean lBumper = Xbox.getBButton();
        if (lBumper){
          modeNum++;
        }
/* 
    if(modeNum%Constants.LED_MODES_LBUMPER_PRESS==0){
      ledsSubsystem.setColor(0, 0, 0);
      ledsSubsystem.setColumnRainbow(firstColor%180,255,255);
      firstColor++;
    }

    if(modeNum%Constants.LED_MODES_LBUMPER_PRESS==1){
      ledsSubsystem.setColor(0, 0, 0);
      ledsSubsystem.setInputAngle(angle);
       SmartDashboard.putNumber("angle", angle);
    }

    if(modeNum%Constants.LED_MODES_LBUMPER_PRESS==2){
      ledsSubsystem.setColor(0, 0, 0);
      ledsSubsystem.setGlitterAngle(angle);
       SmartDashboard.putNumber("angle", angle);
    }
 
  /*   if(modeNum%Constants.LED_MODES_LBUMPER_PRESS==3){
      intakeSubsystem.setMotorSpeed(Xbox.getRawAxis(2));
      ledsSubsystem.setRainbowMotorSpeed(intakeSubsystem.getMotorSpeed());
    }

    if(modeNum%Constants.LED_MODES_LBUMPER_PRESS==4){
      intakeSubsystem.setMotorSpeed(Xbox.getRawAxis(2));
      ledsSubsystem2023.setColorMotorSpeed(intakeSubsystem.getMotorSpeed());
    }

    if(modeNum%Constants.LED_MODES_LBUMPER_PRESS==5){
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
        */

        intakeSubsystem.periodic();
      }

    //}

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