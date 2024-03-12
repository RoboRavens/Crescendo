// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.util.Constants.ElbowConstants;
import frc.robot.util.Constants.WristConstants;

public class WristMoveWithJoystickCommand extends Command {
  CommandXboxController controller;

  public WristMoveWithJoystickCommand(CommandXboxController _operatorController) {
    controller = _operatorController;
    addRequirements(Robot.WRIST_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("WristMoveManuallyCommand: initialize");
    Robot.WRIST_SUBSYSTEM.setPowerManually(controller.getRightY() * WristConstants.JOYSTICK_CONTROL_SCALING_FACTOR);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.WRIST_SUBSYSTEM.setPowerManually(controller.getRightY() * WristConstants.JOYSTICK_CONTROL_SCALING_FACTOR);
    Robot.WRIST_SUBSYSTEM.setTargetPosition(Robot.WRIST_SUBSYSTEM.getPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("WristMoveManuallyCommand: end");
    Robot.WRIST_SUBSYSTEM.setPowerManually(0);
    Robot.WRIST_SUBSYSTEM.setTargetPosition(Robot.WRIST_SUBSYSTEM.getPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
