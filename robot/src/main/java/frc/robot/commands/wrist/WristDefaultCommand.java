// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class WristDefaultCommand extends Command {
  private static String DASHBOARD_KEY = "Wrist Target Override Degrees";
  private double _prevDegreesFromDashboard = 0;

  /** Creates a new WristDefaultCommand. */
  public WristDefaultCommand() {
    addRequirements(Robot.WRIST_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("WristDefaultCommand initialize");
    Robot.WRIST_SUBSYSTEM.goToPosition(Robot.WRIST_SUBSYSTEM.getTargetPosition());
    _prevDegreesFromDashboard = Robot.WRIST_SUBSYSTEM.getDegrees();
    SmartDashboard.putNumber(DASHBOARD_KEY, _prevDegreesFromDashboard);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double newTargetDegrees = SmartDashboard.getNumber(DASHBOARD_KEY, _prevDegreesFromDashboard);
    if (_prevDegreesFromDashboard != newTargetDegrees) {
      Robot.WRIST_SUBSYSTEM.goToDegrees(newTargetDegrees);
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
