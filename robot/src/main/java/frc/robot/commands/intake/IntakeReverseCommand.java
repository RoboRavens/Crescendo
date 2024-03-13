package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class IntakeReverseCommand extends Command {

  public IntakeReverseCommand() {
    this.addRequirements(Robot.INTAKE_SUBSYSTEM);
  }

  @Override
  public void initialize() {
    Robot.INTAKE_SUBSYSTEM.reverseIntake();
  }

  @Override
  public void execute() {
    // Any time this command is run, re-indexing will need to take place after.
    Robot.INTAKE_SUBSYSTEM.clearIndexBooleans();
  }

  @Override
  public void end(boolean interrupted) {
    Robot.INTAKE_SUBSYSTEM.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
