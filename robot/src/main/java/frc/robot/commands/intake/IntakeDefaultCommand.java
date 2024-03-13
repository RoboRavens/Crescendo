package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDefaultCommand extends Command {

  public IntakeDefaultCommand() {
    this.addRequirements(Robot.INTAKE_SUBSYSTEM);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // The finish condition is when we have already finished indexing forward, and the shooter no longer has a piece.
    // This means the piece got to the shooter and was then backed out from it.
    if (Robot.INTAKE_SUBSYSTEM.getFinishedIndexingForward() == true && Robot.SHOOTER_SUBSYSTEM.hasPiece() == false) {
        Robot.INTAKE_SUBSYSTEM.setHasIndexedPiece(true);
    }

    // No need to index if the piece has already been indexed, or there's no piece at all.
    if (Robot.INTAKE_SUBSYSTEM.getHasIndexedPiece() == true || Robot.INTAKE_SUBSYSTEM.intakeHasPiece() == false) {
        Robot.INTAKE_SUBSYSTEM.stop();
    }
    else {
        // If we've made it here, the intake has a piece and indexing must take place.
        // It will either be forward or backward based on whether or not the shooter has a piece.
        if (Robot.SHOOTER_SUBSYSTEM.hasPiece()) {
            Robot.INTAKE_SUBSYSTEM.setFinishedIndexingForward(true);
            Robot.INTAKE_SUBSYSTEM.indexPieceBackward();
        }
        else {
            Robot.INTAKE_SUBSYSTEM.indexPieceForward();
        }
    }
  }

  @Override
  public void end(boolean interrupted) {
    Robot.INTAKE_SUBSYSTEM.stop();
  }

  @Override
  public boolean isFinished() {
    // TODO: Implement logic to stop the intake once detected by the beambreak
    return false;
  }
}
