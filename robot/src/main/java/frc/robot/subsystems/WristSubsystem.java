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
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.util.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase {
  private TalonFX _wristRotationMotor = new TalonFX(RobotMap.WRIST_ROTATION_MOTOR);

  private Slot0Configs _pidConfig = WristConstants.getSlot0Configs();
  
  private double targetPosition = 0;
  
  public WristSubsystem() {
    var talonFXConfiguration = new TalonFXConfiguration();
    talonFXConfiguration.MotionMagic.MotionMagicAcceleration = 100;
    talonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = 20;
    talonFXConfiguration.Slot0 = _pidConfig;

    talonFXConfiguration.Audio.BeepOnBoot = false;
    talonFXConfiguration.Audio.BeepOnConfig = false;

    talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    //talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    //talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 5;
    //talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    //talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -5;

    _wristRotationMotor.getConfigurator().setPosition(0);
    _wristRotationMotor.getConfigurator().apply(talonFXConfiguration);
    this.updateStaticFeedfoward();
  }

  public void setTargetPosition(double position) {
    this.targetPosition = position;
  }

  public double getTargetPosition() {
    return this.targetPosition;
  }

  private void updateStaticFeedfoward() {
    var angle = this.getRadiansFromPosition(this.getPosition());

    SmartDashboard.putNumber("Wrist Angle Degrees", Math.toDegrees(angle));
    double totalAngle = angle + Robot.ELBOW_SUBSYSTEM.getRadians();
    //SmartDashboard.putNumber("Total Angle Radians", totalAngle);
    SmartDashboard.putNumber("Wrist Total Angle Degrees", Math.toDegrees(totalAngle));
    //SmartDashboard.putNumber("Cosine Total Angle", Math.cos(totalAngle));
    //SmartDashboard.putNumber("Wrist Total Angle", Math.toRadians(15)+totalAngle);
    double wristFeedForward =  WristConstants.MOTOR_POWER_FEEDFORWARD_AT_HORIZONTAL * Math.cos(Math.toRadians(15)+totalAngle);
    //SmartDashboard.putNumber("Wrist Feed Forward", wristFeedForward);
    if (_pidConfig.kS != wristFeedForward) {
       _pidConfig.kS = wristFeedForward;
       _wristRotationMotor.getConfigurator().apply(_pidConfig);
    }
  }

  private double getRadiansFromPosition(double position) {
    double unitsTo90 = WristConstants.ENCODER_POSITION_45_FROM_ROBOT_START - WristConstants.ENCODER_POSITION_AT_ROBOT_START;
    double distanceFromHorizontal = ((position - WristConstants.ENCODER_POSITION_AT_ROBOT_START) / unitsTo90);
    double angleInRadians = distanceFromHorizontal * (Math.PI / 4);
    return angleInRadians;
  }

  public double getRadians() {
    double radians = this.getRadiansFromPosition(getPosition());
    return radians;
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
