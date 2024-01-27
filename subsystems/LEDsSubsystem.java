// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDsSubsystem extends SubsystemBase {
  private AddressableLED m_led = new AddressableLED();
  private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer();
  /** Creates a new LEDsSubsystem. */
  public LEDsSubsystem() {
    m_led = new AddressableLED(9);
    
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setlLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
