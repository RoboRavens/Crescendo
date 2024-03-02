// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class WristDefaultCommand extends Command {
  /** Creates a new WristDefaultCommand. */
  public WristDefaultCommand() {
    addRequirements(Robot.WRIST_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        System.out.println("WristDefaultCommand initialize");
    var positionToHold = Robot.WRIST_SUBSYSTEM.getPosition();
    Robot.WRIST_SUBSYSTEM.goToPosition(positionToHold);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("WristDefaultCommand end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
