// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elbow;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ElbowMoveManuallyCommand extends Command {
  double _power;

  public ElbowMoveManuallyCommand(double power) {
    _power = power;
    addRequirements(Robot.ELBOW_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ElbowMoveManuallyCommand: initialize");
    Robot.ELBOW_SUBSYSTEM.setPowerManually(_power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.ELBOW_SUBSYSTEM.setTargetPosition(Robot.ELBOW_SUBSYSTEM.getPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.ELBOW_SUBSYSTEM.setPowerManually(0);
    Robot.ELBOW_SUBSYSTEM.setTargetPosition(Robot.ELBOW_SUBSYSTEM.getPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
