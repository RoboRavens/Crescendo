package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeFeedCommand extends Command {
  private IntakeSubsystem _intakeSubsystem;

  public IntakeFeedCommand(IntakeSubsystem intakeSubsystem) {
    _intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void initialize() {
    _intakeSubsystem.startIntake();
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
