// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.util.Constants.ElbowConstants;

public class WristWaitUntilElbowIsAtPositionCommand extends Command {
  /** Creates a new WristDefaultCommand. */
  double _elbowTargetPosition;
  public WristWaitUntilElbowIsAtPositionCommand(double elbowTargetPositionDegrees) {
    _elbowTargetPosition = ElbowSubsystem.getPositionFromRadians(Math.toRadians(elbowTargetPositionDegrees));
    addRequirements(Robot.WRIST_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("WristWaitUntilElbowIsAtPositionCommand initialize");
    Robot.WRIST_SUBSYSTEM.goToPosition(Robot.WRIST_SUBSYSTEM.getTargetPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("WristWaitUntilElbowIsAtPositionCommand end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(Robot.ELBOW_SUBSYSTEM.getPosition() - _elbowTargetPosition) <= ElbowConstants.IS_AT_SETPOINT_BUFFER;
  }
}
