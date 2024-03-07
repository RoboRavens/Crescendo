// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class IntakeWithSensorCommand extends Command {
  public IntakeWithSensorCommand() {
    this.addRequirements(Robot.INTAKE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("IntakeWithSensorCommand: Init");
    Robot.INTAKE_SUBSYSTEM.startIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.INTAKE_SUBSYSTEM.stop();
    System.out.println("IntakeWithSensorCommand: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.SHOOTER_SUBSYSTEM.hasPiece();
  }
}
