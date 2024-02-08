// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDsSubsystem2 extends SubsystemBase {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private double inputAngle = 0;
  public void setInputAngle(double inputAngle) {
    this.inputAngle = inputAngle;
}

/** Creates a new LEDsSubsystem. */
  public LEDsSubsystem2() {
    m_led = new AddressableLED(9);
    
    m_ledBuffer = new AddressableLEDBuffer(112);
    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    m_led.start();

    for (var i = 0; i < m_ledBuffer.getLength(); i++) {

      
   }
   
   m_led.setData(m_ledBuffer);
   
  }
  
  @Override
  public void periodic() {
    m_ledBuffer.setHSV(0, (int)inputAngle/2, 100, 100);
    m_led.setData(m_ledBuffer);
  }
}
