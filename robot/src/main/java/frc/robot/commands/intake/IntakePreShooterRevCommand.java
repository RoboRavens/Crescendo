package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.util.Constants.IntakeConstants;

public class IntakePreShooterRevCommand extends Command {
  private Timer _timer = new Timer();

  public IntakePreShooterRevCommand() {
    this.addRequirements(Robot.INTAKE_SUBSYSTEM);
  }

  @Override
  public void initialize() {
    _timer.start();
    Robot.INTAKE_SUBSYSTEM.setPowerManually(IntakeConstants.PRE_SHOOTER_REV_REVERSE_SPEED);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    _timer.stop();
    _timer.reset();
    Robot.INTAKE_SUBSYSTEM.stop();
  }

  @Override
  public boolean isFinished() {
    return _timer.get() > .03;
  }
}
