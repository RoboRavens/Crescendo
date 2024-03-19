// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elbow;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ElbowDefaultCommand extends Command {
  private Timer _acquiesceTimer = new Timer();
  private boolean _acquiesced = false;

  /** Creates a new ElbowHoldPositionCommand. */
  public ElbowDefaultCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.ELBOW_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ElbowDefaultCommand: initialize");
    Robot.ELBOW_SUBSYSTEM.goToPosition(Robot.ELBOW_SUBSYSTEM.getTargetPosition());
    _acquiesceTimer.reset();
    _acquiesced = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (_acquiesced == false && _acquiesceTimer.get() > 3) {
      _acquiesced = true;
      Robot.ELBOW_SUBSYSTEM.goToPosition(Robot.ELBOW_SUBSYSTEM.getPosition());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ElbowDefaultCommand end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
