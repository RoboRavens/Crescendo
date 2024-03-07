// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.util.Constants.IntakeConstants;

public class FeedWithSensorCommand extends Command {
  private Timer _timer = new Timer();

  public FeedWithSensorCommand() {
    this.addRequirements(Robot.INTAKE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("FeedWithSensorCommand: Init");
    Robot.INTAKE_SUBSYSTEM.startIntakeFeeder();
    _timer.stop();
    _timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.INTAKE_SUBSYSTEM.stop();
    System.out.println("FeedWithSensorCommand: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Robot.SHOOTER_SUBSYSTEM.hasPiece() == false) {
      _timer.start();
    }

    return _timer.get() >= IntakeConstants.FEEDER_LAUNCH_MOTOR_STOP_BUFFER;
  }
}
