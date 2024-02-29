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
import frc.robot.Constants;
import frc.robot.RobotMap;

public class ElbowSubsystem extends SubsystemBase {
  private TalonFX _elbowRotationMotor = new TalonFX(RobotMap.ELBOW_ROTATION_MOTOR);
  private TalonFX _elbowRotationFollower = new TalonFX(RobotMap.ELBOW_ROTATION_FOLLOWER_MOTOR);
  private DigitalInput _forwardLimitSwitch = new DigitalInput(RobotMap.ELBOW_FORWARD_LIMIT_DIO);

  public ElbowSubsystem() {
    var talonFXConfiguration = new TalonFXConfiguration();
    talonFXConfiguration.MotionMagic.MotionMagicAcceleration = 100;
    talonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = 20;
    talonFXConfiguration.Slot0 = this.GetSlot0Config();

    talonFXConfiguration.Audio.BeepOnBoot = false;
    talonFXConfiguration.Audio.BeepOnConfig = false;

    talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    //talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    //talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 5;
    //talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    //talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -5;

    _elbowRotationMotor.getConfigurator().setPosition(0);
    _elbowRotationMotor.getConfigurator().apply(talonFXConfiguration);

    _elbowRotationFollower.getConfigurator().apply(new AudioConfigs().withBeepOnBoot(false).withBeepOnConfig(false));
    _elbowRotationFollower.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    _elbowRotationFollower.setControl(new Follower(_elbowRotationMotor.getDeviceID(), false));

    var resetPositionCommand = new InstantCommand(() -> {
      var originalPosition = _elbowRotationMotor.getPosition().getValueAsDouble();
      var newPosition = 0;
      System.out.println("elbow position reset from " + originalPosition + " to " + newPosition);
      _elbowRotationMotor.stopMotor();
      _elbowRotationMotor.setPosition(newPosition);
    }, this).ignoringDisable(true);

    new Trigger(() -> _forwardLimitSwitch.get()).onFalse(resetPositionCommand);
  }

  private Slot0Configs GetSlot0Config() {
    var slot0Config = new Slot0Configs();
    slot0Config.kP = Constants.ELBOW_PID.kP;
    slot0Config.kI = Constants.ELBOW_PID.kI;
    slot0Config.kD = Constants.ELBOW_PID.kD;
    var position = _elbowRotationMotor.getPosition().getValueAsDouble();
    var horizontal = -5.27;
    var vertical = -35.51;

    var unitsTo90 = vertical - horizontal;
    var angle = (Math.PI / 2) * ((position - horizontal) / unitsTo90);
    SmartDashboard.putNumber("Elbow Angle Degrees", Math.toDegrees(angle));
    var staticFeedForward = Math.cos(angle) * .025 * -1;
    SmartDashboard.putNumber("Elbow Feedforward", staticFeedForward);

    slot0Config.kS = staticFeedForward;

    return slot0Config;
  }

  public void setPowerManually(double power){
    _elbowRotationMotor.set(power);
  }

  public void goToPosition(double setpoint) {
    System.out.println("goToPosition " + setpoint);
    _elbowRotationMotor.setControl(new MotionMagicVoltage(setpoint));
  }

  public void stopElbowRotation() {
    _elbowRotationMotor.stopMotor();
  }

  @Override
  public void periodic() {
    var pidConfig = GetSlot0Config();
    _elbowRotationMotor.getConfigurator().apply(pidConfig);
    SmartDashboard.putNumber("Elbow RotationMotor pos", _elbowRotationMotor.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("Elbow ForwardLimitSwitch", _forwardLimitSwitch.get());
  }

  public void resetPosition() {
    _elbowRotationMotor.setPosition(0);
  }

  public double getPosition() {
      return _elbowRotationMotor.getPosition().getValueAsDouble();
  }
}
