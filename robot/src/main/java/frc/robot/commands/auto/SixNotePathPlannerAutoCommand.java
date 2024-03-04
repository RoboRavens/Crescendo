// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SixNotePathPlannerAutoCommand extends Command {
  public static Command getAutoMode() {
    Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile("SixNoteTest2");
    Robot.DRIVETRAIN_SUBSYSTEM.resetOdometry(startingPose);
    return new PathPlannerAuto("SixNoteTest2");

    // PathPlannerPath[] paths = new PathPlannerPath[]{
    //   PathPlannerPath.fromPathFile("6Note1"),
    //   PathPlannerPath.fromPathFile("6Note2"),
    //   PathPlannerPath.fromPathFile("6Note3"),
    //   PathPlannerPath.fromPathFile("6Note4"),
    //   PathPlannerPath.fromPathFile("6Note5"),
    // };
    // return Robot.DRIVETRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(paths[0].getTrajectory(new ChassisSpeeds(0, 0, 0), new Rotation2d(0, 0)))
    //   .andThen(AutoBuilder.followPath(paths[0]))
    //   .andThen(new WaitCommand(1))
    //   .andThen(AutoBuilder.followPath(paths[1]))
    //   .andThen(new WaitCommand(1))
    //   .andThen(AutoBuilder.followPath(paths[2]))
    //   .andThen(new WaitCommand(1))
    //   .andThen(AutoBuilder.followPath(paths[3]))
    //   .andThen(new WaitCommand(1))
    //   .andThen(AutoBuilder.followPath(paths[4]))
    //   .andThen(new WaitCommand(1));
  }
}
