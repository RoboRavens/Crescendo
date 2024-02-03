// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class ArmHumanIntakeCommand extends SequentialCommandGroup {
  /** Creates a new ArmHumanIntakeCommand. */
  public ArmHumanIntakeCommand() {
            addCommands(
      new ArmGoToSetpointCommand(Constants.ARM_HUMAN_INTAKE_COMMAND_SETPOINT).withTimeout(2));
    // Use addRequirements() here to declare subsystem dependencies.
  }

}