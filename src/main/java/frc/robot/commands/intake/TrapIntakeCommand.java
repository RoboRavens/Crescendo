package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class TrapIntakeCommand extends Command {
  private IntakeSubsystem _intakeSubsystem;

  public TrapIntakeCommand(IntakeSubsystem intakeSubsystem) {
    _intakeSubsystem = intakeSubsystem;
    this.addRequirements(_intakeSubsystem);
  }

  @Override
  public void initialize() {
    _intakeSubsystem.startTrapIntake();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    _intakeSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
