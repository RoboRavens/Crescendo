// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.WristSubsystem;

public class WristGoToSpeakerAngleCommand extends Command {
  /** Creates a new WristGoToSpeakerAngleCommand. */
  public WristGoToSpeakerAngleCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.WRIST_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shooterAngleRadians = Math.toRadians(Robot.SHOOTER_SUBSYSTEM.getShooterAngleMapDown(Robot.DRIVETRAIN_SUBSYSTEM.getDistanceFromSpeaker()));
    Robot.WRIST_SUBSYSTEM.goToPosition(WristSubsystem.getPositionFromRadians(shooterAngleRadians));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.WRIST_SUBSYSTEM.setPowerManually(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
