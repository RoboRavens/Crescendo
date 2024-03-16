// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ClimberSubsystem;

public class SetClimberToPowerCommand extends Command {
  private DoubleSupplier _power;
  private ClimberSubsystem _climberSubsystem;
  /** Creates a new SetLeftClimberToPower. */
  public SetClimberToPowerCommand(DoubleSupplier power, ClimberSubsystem climberSubsystem) {
    _climberSubsystem = climberSubsystem;
    _power = power;
    addRequirements(_climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _climberSubsystem.setPower(_power.getAsDouble());
    System.out.println(_power.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _climberSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
