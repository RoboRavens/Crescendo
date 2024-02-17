package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeTrapSubsystem;

public class IntakeCommand extends Command {
  private IntakeTrapSubsystem _intakeTrapSubsystem;

  public IntakeCommand(IntakeTrapSubsystem intakeTrapSubsystem) {
    _intakeTrapSubsystem = intakeTrapSubsystem;
    this.addRequirements(_intakeTrapSubsystem);
  }

  @Override
  public void initialize() {
    _intakeTrapSubsystem.startIntake();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    _intakeTrapSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
