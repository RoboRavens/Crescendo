// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.util.Constants.WristConstants;
import frc.util.StateManagement.ArmUpTargetState;

public class WristAngleFromLLTyCommand extends Command {
  /** Creates a new WristAngleFromLLTyCommand. */
  public WristAngleFromLLTyCommand() {
    addRequirements(Robot.WRIST_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.LIMELIGHT_BACK.hasVisionTargetBoolean()) {
      var up = Robot.ARM_UP_TARGET_STATE == ArmUpTargetState.UP;
      var ty = Robot.LIMELIGHT_BACK.getTy();
      var targetAngle = up ? Robot.SHOOTER_SUBSYSTEM.getShooterAngleLLTyMapUp(ty) : Robot.SHOOTER_SUBSYSTEM.getShooterAngleLLTyMapDown(ty);
      Robot.WRIST_SUBSYSTEM.goToDegrees(targetAngle);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    var up = Robot.ARM_UP_TARGET_STATE == ArmUpTargetState.UP;
    var endDegees = up ? WristConstants.DEGREES_DEFENDED_SCORING : WristConstants.DEGREES_FLOOR_PICKUP;
    Robot.WRIST_SUBSYSTEM.goToDegrees(endDegees);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
