// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private CANSparkMax _climberMotor;
  private int _deviceId;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(int deviceID, boolean reverse) {
    _deviceId = deviceID;
    Timer _timer = new Timer();
    _timer.start();
    System.out.println("ClimberSubsystem ID " + deviceID + " starting motor config at time " + _timer.get());

    _climberMotor = new CANSparkMax(deviceID, MotorType.kBrushless);
    System.out.println("ClimberSubsystem ID " + deviceID + " created SparkMax object at time " + _timer.get());

    _climberMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    _climberMotor.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.FORWARD_SOFT_LIMIT);
    _climberMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    _climberMotor.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.REVERSE_SOFT_LIMIT);
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
    if (_deviceId == RobotMap.LEFT_CLIMBER_MOTOR) {
      SmartDashboard.putNumber("Climber Left Position", _climberMotor.getEncoder().getPosition());
    } else {
      SmartDashboard.putNumber("Climber Right Position", _climberMotor.getEncoder().getPosition());
    }
  }
}
