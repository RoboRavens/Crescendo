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
    var talonFXConfiguration = getTalonFXConfigurationObject();

    this.setTargetPosition(WristConstants.ENCODER_POSITION_AT_STARTUP);
    _wristRotationMotor.getConfigurator().setPosition(WristConstants.ENCODER_POSITION_AT_STARTUP);
    _wristRotationMotor.getConfigurator().apply(talonFXConfiguration);
    this.updateStaticFeedfoward();
  }

  public TalonFXConfiguration getTalonFXConfigurationObject() {
    var talonFXConfiguration = new TalonFXConfiguration();
    talonFXConfiguration.MotionMagic.MotionMagicAcceleration = 10;
    talonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = 5;
    talonFXConfiguration.Slot0 = _pidConfig;
    
    talonFXConfiguration.MotorOutput.DutyCycleNeutralDeadband = .01;

    talonFXConfiguration.Audio.BeepOnBoot = false;
    talonFXConfiguration.Audio.BeepOnConfig = false;

    talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // low stator limit will prevent breaking static friction
    talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfiguration.CurrentLimits.StatorCurrentLimit = 40;

    // low supply limit will cap motor velocity
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 10;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonFXConfiguration.CurrentLimits.SupplyCurrentThreshold = 10;
    talonFXConfiguration.CurrentLimits.SupplyTimeThreshold = 0;

    talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 3.12;
    talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -3.47;

    return talonFXConfiguration;
  }

  public void suspendLimits() {
    var talonFXConfiguration = this.getTalonFXConfigurationObject();
    talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    _wristRotationMotor.getConfigurator().apply(talonFXConfiguration);
  }

  public void restoreLimits() {
    var talonFXConfiguration = this.getTalonFXConfigurationObject();
    _wristRotationMotor.getConfigurator().apply(talonFXConfiguration);
  }

  public void offsetByDegrees(double degrees) {
    double offsetPosition = (WristConstants.ENCODER_POSITION_45_FROM_FLOOR_PICKUP / 45) * degrees;
    System.out.print("Changing wrist motor position from " + getPosition());
    double newPosition = getPosition() + offsetPosition;
    System.out.println(" to " + newPosition);
    _wristRotationMotor.getConfigurator().setPosition(newPosition);
  } 

  public void setTargetPosition(double position) {
    this.targetPosition = position;
  }

  public double getTargetPosition() {
    return this.targetPosition;
  }

  private void updateStaticFeedfoward() {
    var angle = this.getRadians();

    SmartDashboard.putNumber("Wrist Angle Degrees", Math.toDegrees(angle));
    double totalAngle = angle + Robot.ELBOW_SUBSYSTEM.getRadians();
    //SmartDashboard.putNumber("Total Angle Radians", totalAngle);
    SmartDashboard.putNumber("Wrist Total Angle Degrees", Math.toDegrees(totalAngle));
    //SmartDashboard.putNumber("Cosine Total Angle", Math.cos(totalAngle));
    //SmartDashboard.putNumber("Wrist Total Angle", Math.toRadians(15)+totalAngle);
    var cosine = Math.cos(Math.toRadians(WristConstants.DEGREES_OFFSET_TO_VERTICAL)+totalAngle);
    double wristFeedForward =  WristConstants.MOTOR_POWER_FEEDFORWARD_AT_HORIZONTAL * cosine;
    SmartDashboard.putNumber("Wrist Cosine", cosine);
    SmartDashboard.putNumber("Wrist Feed Forward", wristFeedForward);
    if (_pidConfig.kS != wristFeedForward) {
       _pidConfig.kS = wristFeedForward;
       _wristRotationMotor.getConfigurator().apply(_pidConfig);
    }
  }

  public static double getRadiansFromPosition(double position) {
    double unitsTo90 = WristConstants.ENCODER_POSITION_45_FROM_FLOOR_PICKUP - WristConstants.ENCODER_POSITION_AT_FLOOR_PICKUP;
    double distanceFromHorizontal = ((position - WristConstants.ENCODER_POSITION_AT_FLOOR_PICKUP) / unitsTo90);
    double angleInRadians = distanceFromHorizontal * (Math.PI / 4);
    return angleInRadians;
  }

  public double getRadians() {
    double radians = WristSubsystem.getRadiansFromPosition(this.getPosition());
    return radians;
  }

  public double getDegrees() {
    return Math.toDegrees(this.getRadians());
  }

  public static double getPositionFromRadians(double angleInRadians) {
    double distanceFromHorizontal =  angleInRadians / (Math.PI / 4);
    double unitsTo90 = WristConstants.ENCODER_POSITION_45_FROM_FLOOR_PICKUP - WristConstants.ENCODER_POSITION_AT_FLOOR_PICKUP;
    double position = (distanceFromHorizontal * unitsTo90) + WristConstants.ENCODER_POSITION_AT_FLOOR_PICKUP;
    return position;
  }

  public void setPowerManually(double power){
    _wristRotationMotor.set(power);
  }

  public void goToPosition(double setpoint) {
    _wristRotationMotor.setControl(new MotionMagicVoltage(setpoint));
    this.setTargetPosition(setpoint);
  }

  public void goToDegrees(double degrees) {
    this.goToPosition(WristSubsystem.getPositionFromRadians(Math.toRadians(degrees)));
  }

  public void incrementTargetPosition() {
    double currentTargetDegrees = WristSubsystem.getDegreesFromPosition(targetPosition);
    double newTargetDegrees = currentTargetDegrees + WristConstants.WRIST_DEGREES_PER_INCREMENT;
    System.out.println("WristSubsystem: incrementTargetPosition - Current target:" + currentTargetDegrees + " New target: " + newTargetDegrees);
    this.setTargetDegrees(newTargetDegrees);
  }

  public void decrementTargetPosition() {
    double currentTargetDegrees = WristSubsystem.getDegreesFromPosition(targetPosition);
    double newTargetDegrees = currentTargetDegrees - WristConstants.WRIST_DEGREES_PER_INCREMENT;
    System.out.println("WristSubsystem: decrementTargetPosition - Current target:" + currentTargetDegrees + " New target: " + newTargetDegrees);
    this.setTargetDegrees(newTargetDegrees);
  }

  public void setTargetDegrees(double targetDegrees) {
    double targetRadians = Math.toRadians(targetDegrees);
    double targetPosition = WristSubsystem.getPositionFromRadians(targetRadians);

    this.setTargetPosition(targetPosition);
  }

  public static double getDegreesFromPosition(double position) {
    return Math.toDegrees(getRadiansFromPosition(position));
  }

  public void stopWristRotation() {
    _wristRotationMotor.stopMotor();
  }

  @Override
  public void periodic() {
    this.updateStaticFeedfoward();
    SmartDashboard.putNumber("Wrist Motor Position", _wristRotationMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Wrist Target Position", this.targetPosition);

    var currentDegrees = this.getDegrees();
    var targetDegrees = WristSubsystem.getDegreesFromPosition(this.targetPosition);
    SmartDashboard.putNumber("Wrist Target Degrees", targetDegrees);
    SmartDashboard.putNumber("Wrist Degree Diff", targetDegrees - currentDegrees);
  }

  public void resetPosition() {
    _wristRotationMotor.setPosition(WristConstants.ENCODER_POSITION_AT_FLOOR_PICKUP);
    this.goToPosition(WristConstants.ENCODER_POSITION_AT_FLOOR_PICKUP);
  }

  public double getPosition() {
    return _wristRotationMotor.getPosition().getValueAsDouble();
  }
}
