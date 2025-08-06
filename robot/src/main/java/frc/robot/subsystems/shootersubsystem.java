// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class shootersubsystem extends SubsystemBase {
    private TalonFX _leftShooterMotor = new TalonFX(RobotMap.SHOOTER_LEFT_MOTOR_CAN_ID);
  private TalonFX _rightShooterMotor = new TalonFX(RobotMap.SHOOTER_RIGHT_MOTOR_CAN_ID);
  /** Creates a new shootersubsystem. */
  public shootersubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shoot() {
    _leftShooterMotor.set(.2);
    _rightShooterMotor.set(.2);
  }

  public void hold() {
    _leftShooterMotor.set(0);
    _rightShooterMotor.set(0);
  }
}
