// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.util.Constants.WristConstants;

public class WristAngleForFeederShotCommand extends Command {
  /** Creates a new WristAngleForFeederShotCommand. */
  public WristAngleForFeederShotCommand() {
    addRequirements(Robot.WRIST_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.WRIST_SUBSYSTEM.goToDegrees(WristConstants.DEGREES_FEEDER_SHOT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.WRIST_SUBSYSTEM.goToDegrees(WristConstants.DEGREES_FLOOR_PICKUP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
