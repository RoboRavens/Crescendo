package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.intake.IntakeCommand;

public class AutoHelpers {
  // Runs the intake until a note is detected, then feeds the note into the shooter IF we are in our alliance zone while starting to follow the next path
  public static Command buildNoteSubCommand(PathPlannerPath[] paths, int pathGroupIndex) {
    return new IntakeCommand(Robot.INTAKE_SUBSYSTEM).until(() -> true) // TODO: replace with: .until(() -> Robot.LOAD_STATE == LoadState.LOADED)
    .andThen(
      new ParallelCommandGroup(
        // TODO: uncomment these lines
        // new WaitUntilCommand(() -> Robot.ZONE_STATE == ZoneState.ALLIANCE_WING)
        //   .andThen(new IntakeFeedCommand(Robot.INTAKE_SUBSYSTEM)),
        AutoBuilder.followPath(paths[pathGroupIndex]) 
      )
    );
  }
}
