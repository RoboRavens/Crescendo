// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.LimbSubsystem;
import frc.robot.util.arm.ArmSetpoint;

public class ArmGoToSetpointCommand extends Command {
  private ArrayList<ArmSetpoint> subSetpoints = new ArrayList<ArmSetpoint>();
  private LimbSubsystem arm = Robot.LIMB_SUBSYSTEM;
  private ArmSetpoint finalSetpoint;
  private int setpointIterator = 0;
  private Timer timer = new Timer();
  private double timeoutSeconds = 0;
  private ArmSetpoint currentSubSetpoint;

  /** Creates a new ArmGoToSetpointCommand. */
  public ArmGoToSetpointCommand(ArmSetpoint setpoint) {
    this.finalSetpoint = setpoint;
    this.currentSubSetpoint = setpoint;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setFinalArmRotationSetpoint(finalSetpoint.getArmRotationSetpoint());
    arm.setFinalWristRotationSetpoint(finalSetpoint.getWristRotationSetpoint());
    //arm.brakeDisable();

    timeoutSeconds = arm.getCommandTimeoutSeconds();

    arm.armMotionMagic();
    arm.wristMotionMagic();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    arm.setArmRotationPosition(finalSetpoint.getArmRotationSetpoint(), Constants.ARM_ROTATION_VELOCITY, Constants.ARM_ROTATION_ACCELERATION);
    arm.setWristRotationPosition(finalSetpoint.getWristRotationSetpoint(), Constants.WRIST_ROTATION_VELOCITY, Constants.WRIST_ROTATION_ACCELERATION);
  }

  public boolean setpointIsFinished(ArmSetpoint setpoint) {
    double rotationError = Math.abs(setpoint.getArmRotationSetpoint() - arm.getArmCurrentRotationNativeUnits());
    double extensionError = Math.abs(setpoint.getWristRotationSetpoint() - arm.getWristCurrentRotationNativeUnits());

    boolean rotationIsFinished = rotationError < Constants.ARM_ROTATION_IS_AT_SETPOINT_MARGIN_ENCODER_TICKS;
    boolean extensionIsFinished = extensionError < Constants.WRIST_ROTATION_IS_AT_SETPOINT_MARGIN_ENCODER_TICKS;

    /*
    SmartDashboard.putBoolean("EXT FIN", extensionIsFinished);
    SmartDashboard.putNumber("EXT error", extensionError);

    SmartDashboard.putBoolean("ROT FIN", rotationIsFinished);
    SmartDashboard.putNumber("ROT error", rotationError);
*/

    return rotationIsFinished && extensionIsFinished;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    boolean timeout = timer.get() >= timeoutSeconds;
    return setpointIsFinished(finalSetpoint);
  }
}
