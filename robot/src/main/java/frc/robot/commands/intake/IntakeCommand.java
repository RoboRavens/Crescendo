package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
  private IntakeSubsystem _intakeSubsystem;

  public IntakeCommand(IntakeSubsystem intakeSubsystem) {
    _intakeSubsystem = intakeSubsystem;
    this.addRequirements(_intakeSubsystem);
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
    // TODO: Implement logic to stop the intake once detected by the beambreak
    return false;
  }
}
