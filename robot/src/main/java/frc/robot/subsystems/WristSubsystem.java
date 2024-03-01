// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.Constants.Constants;
import frc.robot.util.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase {
  private TalonFX _wristRotationMotor = new TalonFX(RobotMap.WRIST_ROTATION_MOTOR);

  public WristSubsystem() {
    var talonFXConfiguration = new TalonFXConfiguration();
    talonFXConfiguration.MotionMagic.MotionMagicAcceleration = 100;
    talonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = 20;
    talonFXConfiguration.Slot0.kP = WristConstants.WRIST_PID.kP;
    talonFXConfiguration.Slot0.kI = WristConstants.WRIST_PID.kI;
    talonFXConfiguration.Slot0.kD = WristConstants.WRIST_PID.kD;

    //talonFXConfiguration.Audio.BeepOnBoot = false;
    //talonFXConfiguration.Audio.BeepOnConfig = false;

    talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    //talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    //talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 5;
    //talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    //talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -5;

    _wristRotationMotor.getConfigurator().setPosition(0);
    _wristRotationMotor.getConfigurator().apply(talonFXConfiguration);
  }

  public void setPowerManually(double power){
    _wristRotationMotor.set(power);
  }

  public void goToPosition(double setpoint) {
    _wristRotationMotor.setControl(new MotionMagicVoltage(setpoint));
  }

  public void stopWristRotation() {
    _wristRotationMotor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist RotationMotor pos", _wristRotationMotor.getPosition().getValueAsDouble());
  }

  public void resetPosition() {
    _wristRotationMotor.setPosition(0);
  }

  public double getPosition() {
    return _wristRotationMotor.getPosition().getValueAsDouble();
  }
}
