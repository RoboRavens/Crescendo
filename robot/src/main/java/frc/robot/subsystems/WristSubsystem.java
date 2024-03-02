// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotMap;
import frc.robot.util.Constants.Constants;
import frc.robot.util.Constants.ElbowConstants;
import frc.robot.util.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase {
  private TalonFX _wristRotationMotor = new TalonFX(RobotMap.WRIST_ROTATION_MOTOR);
  private Slot0Configs _pidConfig = ElbowConstants.getSlot0Configs();

  public WristSubsystem() {
    var talonFXConfiguration = new TalonFXConfiguration();
    talonFXConfiguration.MotionMagic.MotionMagicAcceleration = 100;
    talonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = 20;
    talonFXConfiguration.Slot0.kP = WristConstants.WRIST_PID.kP;
    talonFXConfiguration.Slot0.kI = WristConstants.WRIST_PID.kI;
    talonFXConfiguration.Slot0.kD = WristConstants.WRIST_PID.kD;

    talonFXConfiguration.Audio.BeepOnBoot = false;
    talonFXConfiguration.Audio.BeepOnConfig = false;

    talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    //talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    //talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = WristConstants.FORWARD_SOFT_LIMIT_THRESHOLD;
    //talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    //talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = WristConstants.REVERSE_SOFT_LIMIT_THRESHOLD;

    _wristRotationMotor.getConfigurator().setPosition(WristConstants.ENCODER_POSITION_AT_GROUND_PICKUP);
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

  //change for 45degrees rather then 90
  private double getRadiansFromPosition(double position) {
    double unitsTo45 = WristConstants.ENCODER_POSITION_AT_VERTICAL - WristConstants.ENCODER_POSITION_AT_HORIZONTAL;
    double distanceFromHorizontal = ((position - WristConstants.ENCODER_POSITION_AT_HORIZONTAL) / unitsTo45);
    double angleInRadians = distanceFromHorizontal * (Math.PI / 4);
    return angleInRadians;
  }

  private double getDegreesFromPosition(double position) {
    double unitsTo45 = ElbowConstants.ENCODER_POSITION_AT_VERTICAL - ElbowConstants.ENCODER_POSITION_AT_HORIZONTAL;
    double distanceFromHorizontal = ((position - ElbowConstants.ENCODER_POSITION_AT_HORIZONTAL) / unitsTo45);
    double angleInRadians = distanceFromHorizontal * 45;
    return angleInRadians;
  }

  private double getPositionFromRadians(double angleInRadians) {
    double distanceFromHorizontal =  angleInRadians / (Math.PI / 4);
    double unitsTo45 = WristConstants.ENCODER_POSITION_AT_VERTICAL - WristConstants.ENCODER_POSITION_AT_HORIZONTAL;
    double position = (distanceFromHorizontal * unitsTo45) + WristConstants.ENCODER_POSITION_AT_HORIZONTAL;
    return position;
  }

  public double getPositionFromDegrees(double angleInDegrees) {
    double distanceFromHorizontal = angleInDegrees * 1 / 45 ;
    double unitsTo45 = WristConstants.ENCODER_POSITION_AT_VERTICAL - WristConstants.ENCODER_POSITION_AT_HORIZONTAL;
    double position = (distanceFromHorizontal * unitsTo45) + WristConstants.ENCODER_POSITION_AT_HORIZONTAL;
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
    SmartDashboard.putNumber("Wrist RotationMotor pos", _wristRotationMotor.getPosition().getValueAsDouble());
  }

  public void resetPosition() {
    _wristRotationMotor.setPosition(0);
  }

  public double getPosition() {
    return _wristRotationMotor.getPosition().getValueAsDouble();
  }
}
