// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ElbowSubsystem;


public class ElbowRotateManuallyCommand extends Command {
  private ElbowSubsystem elbow = Robot.ARM_SUBSYSTEM;
  private double voltage = Constants.ELBOW_MANUAL_ROTATION_VOLTAGE;
  /** Creates a new ArmRotateManuallyCommand. */
  public ElbowRotateManuallyCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
