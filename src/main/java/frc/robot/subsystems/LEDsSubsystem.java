// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDsSubsystem extends SubsystemBase {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  public void setInputAngle(double inputAngle) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, (int)inputAngle/2, 255, 255);
    }
     m_led.setData(m_ledBuffer);
  }


    public void setGlitterAngle(double inputAngle) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setHSV(0, (int)inputAngle/2, 255, 255);
        m_led.setData(m_ledBuffer);
      }

    }

  public void setRainbowMotorSpeed(double motorSpeed){
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, (int)((180*motorSpeed)/12000), 255, 255);
    }
    m_led.setData(m_ledBuffer);
  }




  /** Creates a new LEDsSubsystem. */
  public LEDsSubsystem() {
    m_led = new AddressableLED(9);
    
    m_ledBuffer = new AddressableLEDBuffer(117);
    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    m_led.start();

    for (var i = 0; i < m_ledBuffer.getLength(); i++) {

      m_ledBuffer.setRGB(i, 0, 0, 0);
   }
   
   m_led.setData(m_ledBuffer);
   
  }
  
  public void setColor(int r, int g, int b) {
     
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {

     m_ledBuffer.setRGB(i, r, g, b);
   }
    m_led.setData(m_ledBuffer);
  }

  @Override
  public void periodic() {
 

    // This method will be called once per scheduler run
  }
}
