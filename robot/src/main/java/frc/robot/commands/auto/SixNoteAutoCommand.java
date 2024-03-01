// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.IntakeFeedCommand;
import frc.robot.commands.shooter.StartShooterCommand;
import frc.util.StateManagement.LoadState;
import frc.util.StateManagement.ZoneState;

public class SixNoteAutoCommand extends Command {
  private static PathPlannerPath[] paths = new PathPlannerPath[]{
    PathPlannerPath.fromChoreoTrajectory("SixNoteBlue.1"),
    PathPlannerPath.fromChoreoTrajectory("SixNoteBlue.2"),
    PathPlannerPath.fromChoreoTrajectory("SixNoteBlue.3"),
    PathPlannerPath.fromChoreoTrajectory("SixNoteBlue.4"),
    PathPlannerPath.fromChoreoTrajectory("SixNoteBlue.5"),
    PathPlannerPath.fromChoreoTrajectory("SixNoteBlue.6"),
    PathPlannerPath.fromChoreoTrajectory("SixNoteBlue.7"),
    PathPlannerPath.fromChoreoTrajectory("SixNoteBlue.8")
  };

  public static Command getAutoMode() {
    return Robot.DRIVETRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(paths[0].getTrajectory(new ChassisSpeeds(0, 0, 0), new Rotation2d(0, 0)))
      .andThen(
        new ParallelCommandGroup(
          new StartShooterCommand(),
          new ParallelCommandGroup(
            // TODO: Uncomment these lines
            // new WaitUntilCommand(() -> Robot.SHOOTER_SUBSYSTEM.isShooterAtTargetShotVelocity())
              // .andThen(new IntakeFeedCommand(Robot.INTAKE_SUBSYSTEM))
              // .andThen(new IntakeCommand(Robot.INTAKE_SUBSYSTEM)), 
            AutoBuilder.followPath(paths[0])
          )
          .andThen(buildAllianceNoteSubCommand(1)) // Runs the intake for first ground note and runs the next path while shooting the note
          .andThen(buildAllianceNoteSubCommand(2)) // Runs the intake for second ground note and runs the next path while shooting the note
          .andThen(buildAllianceNoteSubCommand(3)) // Runs the intake for third ground note and runs the next path while shooting the note
          .andThen(buildCenterNoteSubCommand(4)) // Runs the intake for the first center note and runs the next path while shooting the note
          .andThen(buildCenterNoteSubCommand(5)) // Runs the intake for the second center note and runs the next path while shooting the note
          .andThen(new IntakeCommand(Robot.INTAKE_SUBSYSTEM)) // Runs the intake for the last note of the auto path (the third center note)
          )
      );
  }

  // Runs the intake until a note is detected, then feeds the note into the shooter while starting to follow the next path
  private static Command buildAllianceNoteSubCommand(int pathGroupIndex) {
    return new IntakeCommand(Robot.INTAKE_SUBSYSTEM).until(() -> true) // TODO: replace with: .until(() -> Robot.LOAD_STATE == LoadState.LOADED)
    .andThen(
      new ParallelCommandGroup(
        // TODO: Uncomment this line
        // new IntakeFeedCommand(Robot.INTAKE_SUBSYSTEM),
        AutoBuilder.followPath(paths[pathGroupIndex]) 
      )
    );
  }

  // Runs the intake until a note is detected, then feeds the note into the shooter IF we are in our alliance zone while starting to follow the next path
  private static Command buildCenterNoteSubCommand(int pathGroupIndex) {
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
