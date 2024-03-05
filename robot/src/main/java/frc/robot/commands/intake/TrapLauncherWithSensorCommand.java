// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.util.Constants.IntakeConstants;

public class TrapLauncherWithSensorCommand extends Command {
  private Timer _timer = new Timer();

  public TrapLauncherWithSensorCommand() {
    this.addRequirements(Robot.INTAKE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.INTAKE_SUBSYSTEM.startIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.INTAKE_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Robot.INTAKE_SUBSYSTEM.intakeHasPiece() == false) {
      _timer.start();
      if (_timer.get() >= IntakeConstants.TRAP_LAUNCH_MOTOR_STOP_BUFFER) {
        _timer.stop();
        _timer.reset();
        return true;
      }
    }
    return false;
  }
}
