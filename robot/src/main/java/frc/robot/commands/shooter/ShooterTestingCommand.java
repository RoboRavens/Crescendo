// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

// This command turns on the shooter motors and automatically stops when
// the robot detects it no longer has a piece
public class ShooterTestingCommand extends Command {
  private static final String SMARTDASHBOARD_POWER_KEY = "Shooter Percent Power";
  public ShooterTestingCommand() {
    this.addRequirements(Robot.SHOOTER_SUBSYSTEM);
    SmartDashboard.setDefaultNumber(SMARTDASHBOARD_POWER_KEY, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var percentPower = SmartDashboard.getNumber(SMARTDASHBOARD_POWER_KEY, 0);
    Robot.SHOOTER_SUBSYSTEM.setPowerManually(percentPower / 100.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.SHOOTER_SUBSYSTEM.stopShooting();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
