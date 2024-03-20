// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private CANSparkMax _climberMotor;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(int deviceID, boolean reverse) {
    Timer _timer = new Timer();
    _timer.start();
    System.out.println("ClimberSubsystem ID " + deviceID + " starting motor config at time " + _timer.get());
    _climberMotor = new CANSparkMax(deviceID, MotorType.kBrushless);
    System.out.println("ClimberSubsystem ID " + deviceID + " created SparkMax object at time " + _timer.get());
    _climberMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    _climberMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    _climberMotor.setSmartCurrentLimit(ClimberConstants.CURRENT_LIMIT);
    System.out.println("ClimberSubsystem ID " + deviceID + " enabled soft limits and current limit at time " + _timer.get());
    _climberMotor.restoreFactoryDefaults();
    System.out.println("ClimberSubsystem ID " + deviceID + " restored factory defaults at time " + _timer.get());
    _climberMotor.setIdleMode(IdleMode.kBrake);
    System.out.println("ClimberSubsystem ID " + deviceID + " set idle mode to brake at time " + _timer.get());
    _climberMotor.setInverted(reverse);
  }

  public void setPower(double power) {
    _climberMotor.set(power);
  }

  public void stop() {
    _climberMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
