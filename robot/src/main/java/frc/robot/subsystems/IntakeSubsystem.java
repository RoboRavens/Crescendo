// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX _intakemotorFx = new TalonFX(RobotMap.INTAKE_MOTOR_TOP_CAN_ID);
  private TalonFX _intakemotorFx2 = new TalonFX(RobotMap.INTAKE_MOTOR_BOTTOM_CAN_ID);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intake() {
    _intakemotorFx.set(.25);
    _intakemotorFx2.set(.25);
  }

  public void eject() {
    _intakemotorFx.set(-.1);
    _intakemotorFx2.set(-.1);
  }

  public void stop() {
    _intakemotorFx.set(0);
    _intakemotorFx2.set(0);
  }

}
