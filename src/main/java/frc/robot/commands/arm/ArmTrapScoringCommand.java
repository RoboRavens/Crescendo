// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class ArmTrapScoringCommand extends SequentialCommandGroup {
  /** Creates a new ArmTrapScoringCommand. */
  public ArmTrapScoringCommand() {
            addCommands(
      new ArmGoToSetpointCommand(Constants.ARM_TRAP_SCORING_COMMAND_SETPOINT).withTimeout(2));
    // Use addRequirements() here to declare subsystem dependencies.
  }

}