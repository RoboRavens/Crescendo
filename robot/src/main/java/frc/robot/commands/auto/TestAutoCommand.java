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
import frc.robot.Robot;

public class TestAutoCommand extends Command {
  public static Command getAutoMode() {
    PathPlannerPath testChoreoPathPlannerPath = PathPlannerPath.fromChoreoTrajectory("TestPath");
    PathPlannerTrajectory testChoreoPathPlannerTrajectory = testChoreoPathPlannerPath.getTrajectory(
        new ChassisSpeeds(0, 0, 0), new Rotation2d(0, 0));
    return Robot.DRIVETRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(testChoreoPathPlannerTrajectory)
      .andThen(AutoBuilder.followPath(testChoreoPathPlannerPath));
  }
}
