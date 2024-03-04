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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.FeedCommand;
import frc.robot.commands.shooter.StartShooterCommand;
import frc.util.StateManagement.LoadState;
import frc.util.StateManagement.ZoneState;

public class SixNoteAutoCommand extends Command {
  private static PathPlannerPath[] paths = new PathPlannerPath[]{
    PathPlannerPath.fromChoreoTrajectory("SixNoteTest.1"),
    PathPlannerPath.fromChoreoTrajectory("SixNoteTest.2"),
    PathPlannerPath.fromChoreoTrajectory("SixNoteTest.3"),
    PathPlannerPath.fromChoreoTrajectory("SixNoteTest.4"),
    PathPlannerPath.fromChoreoTrajectory("SixNoteTest.5"),
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
            .andThen(new WaitCommand(1))
          )
          .andThen(AutoHelpers.buildNoteSubCommand(paths, 1))
          .andThen(new WaitCommand(1)) // Runs the intake for first ground note and runs the next path while shooting the note
          .andThen(AutoBuilder.followPath(paths[2]))
          .andThen(new WaitCommand(1))
          .andThen(AutoHelpers.buildNoteSubCommand(paths, 3))
          .andThen(new WaitCommand(1))
          .andThen(AutoBuilder.followPath(paths[4]))
          .andThen(new WaitCommand(1)) // Runs the intake for second ground note and runs the next path while shooting the note
          .andThen(new IntakeCommand(Robot.INTAKE_SUBSYSTEM)) // Runs the intake for the last note of the auto path (the third center note)
          )
      );
  }
}
