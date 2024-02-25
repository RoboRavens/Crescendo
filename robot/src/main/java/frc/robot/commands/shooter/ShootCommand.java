// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
  /** Creates a new Shoot. */
  private Timer _timer = new Timer();
  public ShootCommand() {
    this.addRequirements(Robot.SHOOTER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.SHOOTER_SUBSYSTEM.startShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.SHOOTER_SUBSYSTEM.stopShooting();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Robot.SHOOTER_SUBSYSTEM.hasPiece() == false){
      _timer.start();
      if(_timer.get() >= ShooterConstants.SHOOTER_STOP_DELAY){
        _timer.stop();
        _timer.reset();
        return true;
      }
    }
    return false;
  }
}
