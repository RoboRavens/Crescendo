// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.LEDsConstants;

public class LEDsSubsystem24 extends SubsystemBase {
  public enum LEDsPattern {
    Solid,
    Blinking,
    Rainbow
  }

  private AddressableLED m_led = new AddressableLED(0);
  boolean m_blinkOn = false;
  Timer m_blinkTimer = new Timer();
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LEDsConstants.TOTAL_LEDS_STRIP_LENGTH);
  double m_rainbowFirstPixelHue = 0;
  int m_rainbowValue = 0;

  private LEDsPattern m_pattern;
  private Color m_colorToBe;
  private Color m_currentColor;

  public LEDsSubsystem24() {
    m_blinkTimer.reset();
    m_blinkTimer.start();

    m_led.setLength(m_ledBuffer.getLength());
    m_led.start();
  }

  @Override
  public void periodic() {
    switch (m_pattern) {
      case Rainbow:
        this.rainbowLeds();
        break;
      case Blinking:
        this.blinkLEDsColor(m_colorToBe, Color.kBlack);
        break;
      case Solid:
        this.ledsSolidColorEfficiently(m_colorToBe);
        break;
      default:
        break;
    }
  }

  public void setPattern(LEDsPattern pattern) {
    m_pattern = pattern;
  }

  public void setColor(Color color) {
    m_colorToBe = color;
  }

  // only updates buffer if color has changed
  private void ledsSolidColorEfficiently(Color color) {
    //if (m_currentColor != color) {
      int r = (int)(color.red * 255.0);
      int g = (int)(color.green * 255.0);
      int b = (int)(color.blue * 255.0);
      SmartDashboard.putString("LED Color", color.toString());
      SmartDashboard.putString("LED RGB", r + ":" + g + ":" + b);
      this.ledsSolidColor(r,g,b);
      m_currentColor = color;
    //}
  }

  // sets all the leds to the same color
  private void ledsSolidColor(int red, int green, int blue) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, red, green, blue);
    }

    m_led.setData(m_ledBuffer);
  }

  private void blinkLEDsColor(Color colorOn, Color colorOff) {
    if (m_blinkTimer.get() >= 0.25) {
      var color = m_blinkOn ? colorOn : colorOff;
      this.ledsSolidColorEfficiently(color);
      m_blinkTimer.reset();
    }
  }

  // sets the leds to a rainbow based off the total length of the leds
  private void rainbowLeds() {
    m_currentColor = null;
    SmartDashboard.putString("LED Color", "rainbow");
    SmartDashboard.putString("LED RGB", "rainbow");
    double hueMin = 110.5;
    double hueMax = 133;
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      final int hue = (int) Math.round((m_rainbowFirstPixelHue + (i * hueMin / m_ledBuffer.getLength())) % hueMax);
      m_ledBuffer.setHSV(i, hue, 255, m_rainbowValue);
    }

    if (m_rainbowValue < 255) {
      m_rainbowValue += 25;
    }
    m_rainbowFirstPixelHue += 0.5;
    m_rainbowFirstPixelHue %= hueMax;

    m_led.setData(m_ledBuffer);
  }

  /*
  public void setInputAngle(double inputAngle) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, (int) inputAngle / 2, 255, 255);
    }
    m_led.setData(m_ledBuffer);
  }

  public void setGlitterAngle(double inputAngle) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(0, (int) inputAngle / 2, 255, 255);
      m_led.setData(m_ledBuffer);
    }

  }

  public void setRainbowMotorSpeed(double motorSpeed) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, (int) ((180 * motorSpeed) / 12000), 255, 255);
    }
    m_led.setData(m_ledBuffer);
  }

  public void setColumnHSV(int n, int h, int s, int v) {
    for (int i = 1; i < m_ledBuffer.getLength(); i++) {
      if (i % LEDsConstants.LEDS_IN_ROW == n) {
        if (i / LEDsConstants.LEDS_IN_ROW % 2 == 0) {
          m_ledBuffer.setHSV(i, h, s, v);
        } else {
          m_ledBuffer.setHSV(
              ((i / LEDsConstants.LEDS_IN_ROW + 1) * LEDsConstants.LEDS_IN_ROW - (i % LEDsConstants.LEDS_IN_ROW)) - 1,
              h, s, v);
        }
      }
    }
    m_led.setData(m_ledBuffer);
  }

  public void setColumnRainbow(int h, int s, int v) {
    for (int i = 1; i <= 28; i++) {
      setColumnHSV(i, h + (90 * i / 28), s, v);
    }
  }

  public void setColor(int r, int g, int b) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }

    m_led.setData(m_ledBuffer);
  }

  // sets every other led to a different color
  public void ledsAlternatingColors(int redInC1, int greenInC1, int blueInC1, int redInC2, int greenInC2,
      int blueInC2) {
    for (int i = 0; i < m_ledBuffer.getLength(); i = i + 2) {
      m_ledBuffer.setRGB(i, redInC1, greenInC1, blueInC1);
      m_ledBuffer.setRGB(i + 1, redInC2, greenInC2, blueInC2);
    }

    setData(m_ledBuffer);
  }

  public void ledsBlinkThenStop(int redInC1, int greenInC1, int blueInC1, int redInC2, int greenInC2, int blueInC2) {
    int timesBlinked = 0;

    if (m_blinkTimer.get() >= 0.25) {
      // toggle state
      if (m_blinkOn) {
        m_blinkOn = false;
      } else {
        m_blinkOn = true;
      }
      m_blinkTimer.reset();
    }

    if (timesBlinked < 10) {
      if (m_blinkOn == true) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setRGB(i, redInC1, greenInC1, blueInC1);
        }
        timesBlinked += 1;
      } else {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setRGB(i, redInC2, greenInC2, blueInC2);
        }
        timesBlinked += 1;
      }
      setData(m_ledBuffer);
    } else {
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, redInC1, greenInC1, blueInC1);
      }

      setData(m_ledBuffer);
    }
  }

  // blinks the LEDs on and off every second
  public void ledsBlinkColors(int redInC1, int greenInC1, int blueInC1, int redInC2, int greenInC2, int blueInC2) {
    if (m_blinkTimer.get() >= 1) {
      // toggle state
      if (m_blinkOn) {
        m_blinkOn = false;
      } else {
        m_blinkOn = true;
      }
      m_blinkTimer.reset();
    }

    if (m_blinkOn == true) {
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, redInC1, greenInC1, blueInC1);
      }
    } else {
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, redInC2, greenInC2, blueInC2);
      }
    }
    setData(m_ledBuffer);
  }

  public void setColorMotorSpeed(double motorSpeed) {
    if (motorSpeed > IntakeConstants.MIN_INTAKE_RPM && motorSpeed < IntakeConstants.MAX_INTAKE_RPM) {
      ledsBlinkColors(0, 255, 0, 0, 0, 0);
    } else {
      ledsBlinkColors(255, 0, 0, 0, 0, 0);
    }
  }

  // sets different LEDs sections different colors
  public void ledsSection(int redInC1, int greenInC1, int blueInC1, int redInC2, int greenInC2, int blueInC2,
      int redInC3, int greenInC3, int blueInC3, int redInC4, int greenInC4, int blueInC4) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      if (i < LEDsConstants.END_OF_FIRST_LEDS_SECTION) {
        m_ledBuffer.setRGB(i, redInC1, greenInC1, blueInC1);
      }
      if (i < LEDsConstants.END_OF_SECOND_LEDS_SECTION && i > LEDsConstants.END_OF_FIRST_LEDS_SECTION) {
        m_ledBuffer.setRGB(i, redInC2, greenInC2, blueInC2);
      }
    }

    setData(m_ledBuffer);
  }
  */
}
