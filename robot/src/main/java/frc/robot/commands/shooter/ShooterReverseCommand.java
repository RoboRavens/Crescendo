// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.util.StateManagement.ShooterRevTargetState;

public class ShooterReverseCommand extends Command {
  /** Used in autonomous modes to continuously run the shooter */
  public ShooterReverseCommand() {
    this.addRequirements(Robot.SHOOTER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ShooterReverseCommand: Init");
    Robot.SHOOTER_SUBSYSTEM.setPowerManually(-0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.SHOOTER_SUBSYSTEM.stopShooting();
    System.out.println("ShooterReverseCommand: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
