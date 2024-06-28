// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.util.Constants.WristConstants;

public class WristGoToDegreesCommand extends Command {
  private double _targetDegrees;
  /** Creates a new WristGoToPositionCommand. */
  public WristGoToDegreesCommand(double degrees) {
    _targetDegrees = degrees;
    addRequirements(Robot.WRIST_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("WristGoToDegreesCommand: initialize");
    Robot.WRIST_SUBSYSTEM.goToDegrees(_targetDegrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("WristGoToDegreesCommand: end" + (interrupted ? " interrupted": ""));
    Robot.WRIST_SUBSYSTEM.stopWristRotation();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double wristDiff = Math.abs(Robot.WRIST_SUBSYSTEM.getDegrees() - _targetDegrees);
    if(wristDiff <= WristConstants.IS_AT_SETPOINT_BUFFER){
      return true;
    }

    return false;
  }
}
