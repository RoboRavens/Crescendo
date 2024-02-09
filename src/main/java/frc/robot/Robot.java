// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDsSubsystem24;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  public XboxController Xbox = new XboxController(0);

  public LEDsSubsystem24 ledsSubsystem24 = new LEDsSubsystem24();
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
  
  
  }


  @Override
  public void robotPeriodic(){
    double xAxis = Xbox.getRawAxis(0);
    SmartDashboard.putNumber("xstick", xAxis);

    double yAxis = Xbox.getRawAxis(1);
    SmartDashboard.putNumber("ystick", yAxis);

    double angle180 = Math.atan2(yAxis,xAxis) * (180 / Math.PI);
    SmartDashboard.putNumber("angle180", angle180);

    SmartDashboard.putNumber("modenum", modeNum);

    angle = angle180;
    if(angle180 < 0){
      angle = 365 + angle;
    }

     
    

   

      
    boolean lBumper = Xbox.getLeftBumper();
        if (lBumper){
          modeNum++;
        }
 
    if(modeNum%Constants.LED_MODES_LBUMPER_PRESS==0){
      ledsSubsystem24.setColor(0, 0, 0);
      if(firstColor%5==0){
      ledsSubsystem24.setColumnRainbow(firstColor%180,255,255);
      }
      firstColor += 1;
      ledsSubsystem24.setColumnRainbow(firstColor%180,255,255);
      firstColor++;
    }

    if(modeNum%Constants.LED_MODES_LBUMPER_PRESS==1){
      ledsSubsystem24.setColor(0, 0, 0);
      ledsSubsystem24.setInputAngle(angle);
       SmartDashboard.putNumber("angle", angle);
    }

    if(modeNum%Constants.LED_MODES_LBUMPER_PRESS==2){
      ledsSubsystem24.setColor(0, 0, 0);
      //ledsSubsystem24.setGlitterAngle(angle);
      // SmartDashboard.putNumber("angle", angle);
    }
 
    if(modeNum%Constants.LED_MODES_LBUMPER_PRESS==3){
      ledsSubsystem24.setColor(0, 0, 0);
      //intakeSubsystem.setMotorSpeed(Xbox.getRawAxis(2));
      //ledsSubsystem24.setRainbowMotorSpeed(intakeSubsystem.getMotorSpeed());
    }

    if(modeNum%Constants.LED_MODES_LBUMPER_PRESS==4){
      ledsSubsystem24.setColor(0, 0, 0);
      //intakeSubsystem.setMotorSpeed(Xbox.getRawAxis(2));
      //ledsSubsystem24.setColorMotorSpeed(intakeSubsystem.getMotorSpeed());
    }

    if(modeNum%Constants.LED_MODES_LBUMPER_PRESS==5){
        ledsSubsystem24.setColor(0, 0, 0);
          boolean aButton = Xbox.getAButton();
        if (aButton) {
          System.out.println("a button pressed, green");
          ledsSubsystem24.setColor(0, 255, 0);
        }

        boolean bButton = Xbox.getBButton();
        if (bButton) {
          System.out.println("b button pressed, red");
          ledsSubsystem24.setColor(255, 0, 0);
        }

        boolean xButton = Xbox.getXButton();
        if (xButton) {
          System.out.println("x button pressed, blue");
          ledsSubsystem24.setColor(0, 0, 255);
        }

        boolean yButton = Xbox.getYButton();
        if (yButton) {
          System.out.println("y button pressed, yellow");
          ledsSubsystem24.setColor(127, 127, 0);
        }

        boolean rightBumper = Xbox.getRightBumper();
        if (rightBumper) {
          System.out.println("rBumper pressed, no color");
          ledsSubsystem24.setColor(0, 0, 0);
        }
      }

       // intakeSubsystem.periodic()
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
