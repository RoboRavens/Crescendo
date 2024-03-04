// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class WristGoToPositionCommand extends Command {
  private double _targetPosition;
  /** Creates a new WristGoToPositionCommand. */
  public WristGoToPositionCommand(double position) {
    _targetPosition = position;
    addRequirements(Robot.WRIST_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("WristGoToPositionCommand: initialize");
    Robot.WRIST_SUBSYSTEM.goToPosition(_targetPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("WristGoToPositionCommand: end" + (interrupted ? " interrupted": ""));
    Robot.WRIST_SUBSYSTEM.stopWristRotation();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double wristDiff = Math.abs(Robot.WRIST_SUBSYSTEM.getPosition() - _targetPosition);
    if(wristDiff <= 0.1){
      return true;
    }

    return false;
  }
}
