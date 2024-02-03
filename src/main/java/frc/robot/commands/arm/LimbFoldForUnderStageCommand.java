// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class LimbFoldForUnderStageCommand extends SequentialCommandGroup {
  /** Creates a new ArmGroundPickUpCommand. */
  public LimbFoldForUnderStageCommand() {
        addCommands(
      new WristGoToSetpointCommand(Constants.WRIST_GROUND_INTAKE_COMMAND_SETPOINT),
      new ElbowGoToSetpointCommand(Constants.ELBOW_GROUND_INTAKE_COMMAND_SETPOINT)
    );
      //alter timeout accordingly
    // Use addRequirements() here to declare subsystem dependencies.
  }
}