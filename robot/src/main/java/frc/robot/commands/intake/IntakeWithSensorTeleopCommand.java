// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.util.Constants.IntakeConstants;

public class IntakeWithSensorTeleopCommand extends Command {
  public IntakeWithSensorTeleopCommand() {
    this.addRequirements(Robot.INTAKE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("IntakeWithSensorTeleopCommand: Init");
    Robot.INTAKE_SUBSYSTEM.setPowerManually(IntakeConstants.INTAKE_MOTOR_TELEOP_SPEED * -1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.INTAKE_SUBSYSTEM.stopMotorWithPID();
    System.out.println("IntakeWithSensorTeleopCommand: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.SHOOTER_SUBSYSTEM.hasPiece();
  }
}
