// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.Constants.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private TalonFX _climberMotor;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(int deviceID) {
    _climberMotor = new TalonFX(deviceID);

    _climberMotor.getConfigurator().setPosition(0);
    _climberMotor.getConfigurator().apply(this.getTalonFXConfigurationObject());
  }

  public TalonFXConfiguration getTalonFXConfigurationObject() {
    var talonFXConfiguration = new TalonFXConfiguration();
    talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    return talonFXConfiguration;
  }

  public void setPower(double power) {
    _climberMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
