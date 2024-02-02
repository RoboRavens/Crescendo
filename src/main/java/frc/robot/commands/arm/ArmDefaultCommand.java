// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.LimbSubsystem;

public class ArmDefaultCommand extends Command {

    private LimbSubsystem arm = Robot.LIMB_SUBSYSTEM;
  /** Creates a new ArmDefaultCommand. */
  public ArmDefaultCommand() {
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        if (Robot.ARM_ROTATION_MANUAL_OVERRIDE == false) {
      arm.setArmRotationPosition(arm.getArmRotationFinalTargetNativeUnits(), Constants.ARM_ROTATION_VELOCITY, Constants.ARM_ROTATION_ACCELERATION);
    }
    else {
      arm.stopArmRotation();
    }
        if (Robot.WRIST_ROTATION_MANUAL_OVERRIDE == false) {
      arm.setWristRotationPosition(arm.getWristRotationFinalTargetNativeUnits(), Constants.WRIST_ROTATION_VELOCITY, Constants.WRIST_ROTATION_ACCELERATION);
    }
    else {
      arm.stopWristRotation();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
