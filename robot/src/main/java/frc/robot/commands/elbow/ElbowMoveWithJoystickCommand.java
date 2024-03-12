// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elbow;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.util.Constants.ElbowConstants;

public class ElbowMoveWithJoystickCommand extends Command {
  CommandXboxController controller;

  public ElbowMoveWithJoystickCommand(CommandXboxController _operatorController) {
    controller = _operatorController;
    addRequirements(Robot.ELBOW_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ElbowMoveManuallyCommand: initialize");
    Robot.ELBOW_SUBSYSTEM.setPowerManually(controller.getLeftY() * ElbowConstants.JOYSTICK_CONTROL_SCALING_FACTOR);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.ELBOW_SUBSYSTEM.setPowerManually(controller.getLeftY() * ElbowConstants.JOYSTICK_CONTROL_SCALING_FACTOR);
    Robot.ELBOW_SUBSYSTEM.setTargetPosition(Robot.ELBOW_SUBSYSTEM.getPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.ELBOW_SUBSYSTEM.setPowerManually(0);
    Robot.ELBOW_SUBSYSTEM.setTargetPosition(Robot.ELBOW_SUBSYSTEM.getPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
