// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeTrapSubsystem;

public class IntakeCommand extends Command {
  /** Creates a new Intake. */
  private IntakeTrapSubsystem _intakeTrapSubsystem;

  public IntakeCommand(IntakeTrapSubsystem intakeTrapSubsystem) {
    _intakeTrapSubsystem = intakeTrapSubsystem;
    this.addRequirements(_intakeTrapSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _intakeTrapSubsystem.startIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _intakeTrapSubsystem.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _intakeTrapSubsystem.intakeHasPiece();
  }
}
