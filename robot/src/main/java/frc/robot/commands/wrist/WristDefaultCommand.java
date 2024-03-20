// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.util.arm.LimbSetpoint;

public class WristDefaultCommand extends Command {
  private Timer _acquiesceTimer = new Timer();
  private boolean _acquiesced = false;

  /** Creates a new WristDefaultCommand. */
  public WristDefaultCommand() {
    addRequirements(Robot.WRIST_SUBSYSTEM);
    _acquiesceTimer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("WristDefaultCommand initialize");
    Robot.WRIST_SUBSYSTEM.goToPosition(Robot.WRIST_SUBSYSTEM.getTargetPosition());
    _acquiesceTimer.reset();
    _acquiesced = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (_acquiesced == false && _acquiesceTimer.get() > 3) {
      _acquiesced = true;
      if (Robot.WRIST_SUBSYSTEM.getTargetPosition() == LimbSetpoint.GROUND_PICKUP.getWristRotationPosition()) {
        // Robot.WRIST_SUBSYSTEM.stopWristRotation();
      }
    }
  }

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
