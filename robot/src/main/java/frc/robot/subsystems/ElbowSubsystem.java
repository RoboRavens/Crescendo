// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotMap;
import frc.robot.util.Constants.ElbowConstants;

public class ElbowSubsystem extends SubsystemBase {
  private TalonFX _elbowRotationMotor = new TalonFX(RobotMap.ELBOW_ROTATION_MOTOR);
  private TalonFX _elbowRotationFollower = new TalonFX(RobotMap.ELBOW_ROTATION_FOLLOWER_MOTOR);
  private DigitalInput _forwardLimitSwitch = new DigitalInput(RobotMap.ELBOW_FORWARD_LIMIT_DIO);
  private boolean _atCurrentLimit = false;

  private Slot0Configs _pidConfig = ElbowConstants.getSlot0Configs();

  public ElbowSubsystem() {
    var talonFXConfiguration = new TalonFXConfiguration();
    talonFXConfiguration.MotionMagic.MotionMagicAcceleration = 100;
    talonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = 20;
    talonFXConfiguration.Slot0 = _pidConfig;

    talonFXConfiguration.Audio.BeepOnBoot = false;
    talonFXConfiguration.Audio.BeepOnConfig = false;

    talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // low stator limit will prevent breaking static friction
    talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfiguration.CurrentLimits.StatorCurrentLimit = 25;

    // low supply limit will cap motor velocity
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 10;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonFXConfiguration.CurrentLimits.SupplyCurrentThreshold = 10;
    talonFXConfiguration.CurrentLimits.SupplyTimeThreshold = 0;

    //talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    //talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 5;
    //talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    //talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -5;

    _elbowRotationMotor.getConfigurator().setPosition(ElbowConstants.ENCODER_POSITION_AT_GROUND_PICKUP);
    _elbowRotationMotor.getConfigurator().apply(talonFXConfiguration);
    this.updateStaticFeedfoward();

    _elbowRotationFollower.getConfigurator().apply(new AudioConfigs().withBeepOnBoot(false).withBeepOnConfig(false));
    _elbowRotationFollower.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    _elbowRotationFollower.setControl(new Follower(_elbowRotationMotor.getDeviceID(), false));

    var resetPositionCommand = new InstantCommand(() -> {
      var originalPosition = _elbowRotationMotor.getPosition().getValueAsDouble();
      var newPosition = ElbowConstants.ENCODER_POSITION_AT_GROUND_PICKUP;
      System.out.println("ElbowSubsystem: position reset from " + originalPosition + " to " + newPosition);
      _elbowRotationMotor.stopMotor();
      _elbowRotationMotor.setPosition(newPosition);
    }, this).ignoringDisable(true);

    new Trigger(() -> _forwardLimitSwitch.get()).onFalse(resetPositionCommand);
  }

  private void updateStaticFeedfoward() {
    var angle = this.getRadiansFromPosition(this.getPosition());
    var staticFeedForward = this.getStaticFeedforwardFromRadians(angle);

    SmartDashboard.putNumber("Elbow Angle Degrees", Math.toDegrees(angle));
    SmartDashboard.putNumber("Elbow Feedforward", staticFeedForward);
    if (_pidConfig.kS != staticFeedForward) {
      _pidConfig.kS = staticFeedForward;
      _elbowRotationMotor.getConfigurator().apply(_pidConfig);
    }
  }

  private double getStaticFeedforwardFromRadians(double angle) {
    var staticFeedForward = Math.cos(angle) * ElbowConstants.MOTOR_POWER_FEEDFORWARD_AT_HORIZONTAL * ElbowConstants.MOTOR_POWER_DIRECTION_TO_GO_UP_FROM_HORIZONTAL;
    return staticFeedForward;
  }

  private double getRadiansFromPosition(double position) {
    double unitsTo90 = ElbowConstants.ENCODER_POSITION_AT_VERTICAL - ElbowConstants.ENCODER_POSITION_AT_HORIZONTAL;
    double distanceFromHorizontal = ((position - ElbowConstants.ENCODER_POSITION_AT_HORIZONTAL) / unitsTo90);
    double angleInRadians = distanceFromHorizontal * (Math.PI / 2);
    return angleInRadians;
  }

  public double getRadians() {
    double radians = this.getRadiansFromPosition(this.getPosition());
    return radians;
  }

  private double getPositionFromRadians(double angleInRadians) {
    double distanceFromHorizontal =  angleInRadians / (Math.PI / 2);
    double unitsTo90 = ElbowConstants.ENCODER_POSITION_AT_VERTICAL - ElbowConstants.ENCODER_POSITION_AT_HORIZONTAL;
    double position = (distanceFromHorizontal * unitsTo90) + ElbowConstants.ENCODER_POSITION_AT_HORIZONTAL;
    return position;
  }

  public void setPowerManually(double power){
    _elbowRotationMotor.set(power);
  }

  public void goToPosition(double setpoint) {
    System.out.println("ElbowSubsystem: goToPosition " + setpoint);
    _elbowRotationMotor.setControl(new MotionMagicVoltage(setpoint));
  }

  public void stopElbowRotation() {
    _elbowRotationMotor.stopMotor();
  }

  @Override
  public void periodic() {
    this.updateStaticFeedfoward();
    SmartDashboard.putNumber("Elbow RotationMotor pos", _elbowRotationMotor.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("Elbow ForwardLimitSwitch", _forwardLimitSwitch.get());
    updateCurrentLimitVariable();
  }
  
  private void updateCurrentLimitVariable() {
    if (true) {
      _atCurrentLimit = true;
    }
    else {
      _atCurrentLimit = false;
    }
  }

  public double getPosition() {
      return _elbowRotationMotor.getPosition().getValueAsDouble();
  }
}
