// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class LEDsSubsystem24 extends SubsystemBase {
  private AddressableLED m_led= new AddressableLED(9);
  boolean blinkOn = false;
  Timer timer = new Timer();
  //AddressableLED m_light = new AddressableLED(9);
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(Constants.TOTAL_LEDS_STRIP_LENGTH);
  double m_rainbowFirstPixelHue = 0;
  int m_value = 0;


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

  public void setRainbowMotorSpeed(double motorSpeed) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, (int)((180*motorSpeed)/12000), 255, 255);
    }
    m_led.setData(m_ledBuffer);
  }

  public void setColumnHSV(int n, int h, int s, int v) {
    for(int i = 1; i < m_ledBuffer.getLength(); i++) {
        if(i%Constants.LEDS_IN_ROW==n) {
          if(i/Constants.LEDS_IN_ROW%2==0){
            m_ledBuffer.setHSV(i, h, s, v);
          }
          else{
            m_ledBuffer.setHSV(((i/Constants.LEDS_IN_ROW+1) * Constants.LEDS_IN_ROW-(i % Constants.LEDS_IN_ROW))-1, h, s, v);
          }
        }
    }
    m_led.setData(m_ledBuffer);
  }

  public void setColumnRainbow(int h, int s, int v) {
    for(int i=1; i<=28; i++){
      setColumnHSV(i, h+(90*i/28),s,v);
    }
  }
  
  public LEDsSubsystem24() {
    
    timer.reset();
    timer.start();

    m_led.setLength(m_ledBuffer.getLength());
    m_led.start();
  }

  public void setColor(int r, int g, int b) {
     
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {

     m_ledBuffer.setRGB(i, r, g, b);
   }
    m_led.setData(m_ledBuffer);
  }

  public void setData(AddressableLEDBuffer ledBuffer) {
    m_led.setData(ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

    //sets every other led to a different color
    public void ledsAlternatingColors(int redInC1, int greenInC1, int blueInC1, int redInC2, int greenInC2, int blueInC2) {
      for (int i = 0; i < m_ledBuffer.getLength(); i = i + 2) {
        m_ledBuffer.setRGB(i, redInC1, greenInC1, blueInC1);
        m_ledBuffer.setRGB(i + 1, redInC2, greenInC2, blueInC2);
      }
  
      setData(m_ledBuffer);
    }

  public void ledsBlinkThenStop(int redInC1, int greenInC1, int blueInC1, int redInC2, int greenInC2, int blueInC2) {
   int timesBlinked = 0;

    if(timer.get() >= 0.25) {
      // toggle state
      if (blinkOn) {
        blinkOn = false;
      } else {
        blinkOn = true;
      }
      timer.reset();
    }
    
    if(timesBlinked < 10){
      if (blinkOn == true) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setRGB(i, redInC1, greenInC1, blueInC1);
        }
        timesBlinked += 1;
      } 
      else{
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setRGB(i, redInC2, greenInC2, blueInC2);
        }
        timesBlinked += 1;
      }
      setData(m_ledBuffer);
    }
    else{
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, redInC1, greenInC1, blueInC1);
      }
  
      setData(m_ledBuffer);
    }
  }

  //blinks the LEDs on and off every second
  public void ledsBlinkColors(int redInC1, int greenInC1, int blueInC1, int redInC2, int greenInC2, int blueInC2) {
    if(timer.get() >= 1) {
      // toggle state
      if (blinkOn) {
        blinkOn = false;
      } else {
        blinkOn = true;
      }
      timer.reset();
    }

    if (blinkOn == true) {
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, redInC1, greenInC1, blueInC1);
      }
    } 
    else{
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, redInC2, greenInC2, blueInC2);
      }
    }
    setData(m_ledBuffer);
  }

    //sets the leds to a rainbow based off the total length of the leds
    public void rainbowLeds() {
      double hueMin = 110.5;
      double hueMax = 133;
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        final int hue = (int) Math.round((m_rainbowFirstPixelHue + (i * hueMin / m_ledBuffer.getLength())) % hueMax);
         m_ledBuffer.setHSV(i, hue, 255, m_value);
       }
       
       if(m_value < 255){
        m_value += 25;
       }
       m_rainbowFirstPixelHue += 0.5;
       m_rainbowFirstPixelHue %= hueMax;
       
      setData(m_ledBuffer);
    }


    public void setColorMotorSpeed(double motorSpeed){
          if(motorSpeed>Constants.MIN_INTAKE_RPM&&motorSpeed<Constants.MAX_INTAKE_RPM){
            ledsBlinkColors(0,255,0,0,0,0);
          }
          else{
            ledsBlinkColors(255,0,0,0,0,0);
          }
      }

  //sets different LEDs sections different colors
  public void ledsSection(int redInC1, int greenInC1, int blueInC1, int redInC2, int greenInC2, int blueInC2, int redInC3, int greenInC3, int blueInC3, int redInC4, int greenInC4, int blueInC4) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      if(i < Constants.END_OF_FIRST_LEDS_SECTION) {
        m_ledBuffer.setRGB(i, redInC1, greenInC1, blueInC1);
      }
      if(i < Constants.END_OF_SECOND_LEDS_SECTION && i > Constants.END_OF_FIRST_LEDS_SECTION) {
        m_ledBuffer.setRGB(i, redInC2, greenInC2, blueInC2);
      }
      if(i < Constants.END_OF_THIRD_LEDS_SECTION && i > Constants.END_OF_SECOND_LEDS_SECTION) {
        m_ledBuffer.setRGB(i, redInC3, greenInC3, blueInC3);
      }
      if(i < Constants.END_OF_FOURTH_LEDS_SECTION && i > Constants.END_OF_THIRD_LEDS_SECTION) {
        m_ledBuffer.setRGB(i, redInC4, greenInC4, blueInC4);
      }
    }

    setData(m_ledBuffer);
  }

  //sets all the leds to the same color
  public void ledsSolidColor(int red, int green, int blue) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, red, green, blue);
    }

    setData(m_ledBuffer);
  }
}
