// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class LimbUnderSpeakerScoringCommand extends SequentialCommandGroup {
  /** Creates a new ArmSpeakerScoringCommand. */
  public LimbUnderSpeakerScoringCommand() {
                addCommands(
      new WristGoToSetpointCommand(Constants.WRIST_UNDER_SPEAKER_SCORING_COMMAND_SETPOINT),
      new ElbowGoToSetpointCommand(Constants.ELBOW_UNDER_SPEAKER_SCORING_COMMAND_SETPOINT)
    );
      //alter timeout accordingly
  }
}