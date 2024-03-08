// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elbow;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ElbowOffsetCommand extends Command {
  private double _degrees;
  /** Creates a new ElbowOffset. */
  public ElbowOffsetCommand(double degrees) {
    _degrees = degrees;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.ELBOW_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.ELBOW_SUBSYSTEM.offsetByDegrees(_degrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
