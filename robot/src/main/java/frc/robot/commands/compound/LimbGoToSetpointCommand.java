// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.util.arm.LimbSetpoint;

public class LimbGoToSetpointCommand extends Command {
  /** Creates a new LimbGoToSetpoint. */
  private LimbSetpoint _targetLimbSetPoint;
  public LimbGoToSetpointCommand(LimbSetpoint targetLimbSetPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    _targetLimbSetPoint = targetLimbSetPoint;
    addRequirements(Robot.ELBOW_SUBSYSTEM, Robot.WRIST_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("LimbGoToSetpointCommand: initialize");
    Robot.ELBOW_SUBSYSTEM.goToPosition(_targetLimbSetPoint.getElbowRotationPosition());
    Robot.WRIST_SUBSYSTEM.goToPosition(_targetLimbSetPoint.getWristRotationPosition());
    String name = _targetLimbSetPoint.getName();
    System.out.println(name);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.ELBOW_SUBSYSTEM.setPowerManually(0);
    Robot.WRIST_SUBSYSTEM.setPowerManually(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double elbowDiff = Math.abs(Robot.ELBOW_SUBSYSTEM.getPosition() - _targetLimbSetPoint.getElbowRotationPosition());
    double wristDiff = Math.abs(Robot.WRIST_SUBSYSTEM.getPosition() - _targetLimbSetPoint.getWristRotationPosition());
    if(elbowDiff <= 0.1 && wristDiff <= 0.1){
      return true;
    }
    return false;
  }
}
