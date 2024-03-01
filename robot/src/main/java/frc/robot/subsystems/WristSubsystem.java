// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase {
  private TalonFX _wristRotationMotor = new TalonFX(RobotMap.WRIST_ROTATION_MOTOR);

  private Slot0Configs _pidConfig = WristConstants.getSlot0Configs();
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
    this.updateStaticFeedfoward();
  }

  private void updateStaticFeedfoward() {
    var angle = this.getRadiansFromPosition(this.getPosition());
    var staticFeedForward = this.getStaticFeedforwardFromRadians(angle);

    SmartDashboard.putNumber("Wrist Angle Degrees", Math.toDegrees(angle));
    SmartDashboard.putNumber("Wrist Feedforward", staticFeedForward);
    if (_pidConfig.kS != staticFeedForward) {
      _pidConfig.kS = staticFeedForward;
      _wristRotationMotor.getConfigurator().apply(_pidConfig);
    }
  }

  private double getStaticFeedforwardFromRadians(double angle) {
    var staticFeedForward = Math.cos(angle) * WristConstants.MOTOR_POWER_FEEDFORWARD_AT_HORIZONTAL * WristConstants.MOTOR_POWER_DIRECTION_TO_GO_UP_FROM_HORIZONTAL;
    return staticFeedForward;
  }

  private double getRadiansFromPosition(double position) {
    double unitsTo90 = WristConstants.ENCODER_POSITION_45_FROM_ROBOT_START - WristConstants.ENCODER_POSITION_AT_ROBOT_START;
    double distanceFromHorizontal = ((position - WristConstants.ENCODER_POSITION_AT_ROBOT_START) / unitsTo90);
    double angleInRadians = distanceFromHorizontal * (Math.PI / 4);
    return angleInRadians;
  }

  private double getPositionFromRadians(double angleInRadians) {
    double distanceFromHorizontal =  angleInRadians / (Math.PI / 4);
    double unitsTo90 = WristConstants.ENCODER_POSITION_45_FROM_ROBOT_START - WristConstants.ENCODER_POSITION_AT_ROBOT_START;
    double position = (distanceFromHorizontal * unitsTo90) + WristConstants.ENCODER_POSITION_AT_ROBOT_START;
    return position;
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
    this.updateStaticFeedfoward();
    SmartDashboard.putNumber("Wrist RotationMotor pos", _wristRotationMotor.getPosition().getValueAsDouble());
  }

  public void resetPosition() {
    _wristRotationMotor.setPosition(0);
  }

  public double getPosition() {
    return _wristRotationMotor.getPosition().getValueAsDouble();
  }
}
