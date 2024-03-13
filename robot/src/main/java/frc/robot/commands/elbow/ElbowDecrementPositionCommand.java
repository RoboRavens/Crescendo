// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elbow;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.util.Constants.ElbowConstants;

public class ElbowDecrementPositionCommand extends Command {
  private double _targetPosition;

  public ElbowDecrementPositionCommand() {
    addRequirements(Robot.ELBOW_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _targetPosition = Robot.ELBOW_SUBSYSTEM.decrementTargetPosition();
    System.out.println("ElbowDecrementPositionCommand: initialize");
    Robot.ELBOW_SUBSYSTEM.goToPosition(_targetPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.ELBOW_SUBSYSTEM.stopElbowRotation();
    System.out.println("ElbowDecrementPositionCommand: end" + (interrupted ? " interrupted": ""));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double elbowDiff = Math.abs(Robot.ELBOW_SUBSYSTEM.getPosition() - _targetPosition);
    if (elbowDiff <= ElbowConstants.IS_AT_SETPOINT_BUFFER){
      return true;
    }

    return false;
  }
}
