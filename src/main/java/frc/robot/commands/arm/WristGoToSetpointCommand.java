// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.util.arm.LimbSetpoint;

public class WristGoToSetpointCommand extends Command {
  //private ArrayList<ArmSetpoint> subSetpoints = new ArrayList<ArmSetpoint>();
  private WristSubsystem arm = Robot.WRIST_SUBSYSTEM;
  private LimbSetpoint finalSetpoint;
  //private int setpointIterator = 0;
  private Timer timer = new Timer();
  private double timeoutSeconds = 0;

  /** Creates a new ArmGoToSetpointCommand. */
  public WristGoToSetpointCommand(LimbSetpoint setpoint) {
    this.finalSetpoint = setpoint;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setFinalWristRotationSetpoint(finalSetpoint.getRotationSetpoint());
    //arm.brakeDisable();

    timeoutSeconds = arm.getCommandTimeoutSeconds();
    arm.wristMotionMagic();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setWristRotationPosition(finalSetpoint.getRotationSetpoint(), Constants.WRIST_ROTATION_VELOCITY, Constants.WRIST_ROTATION_ACCELERATION);
  }

  public boolean setpointIsFinished(LimbSetpoint setpoint) {
    double extensionError = Math.abs(setpoint.getRotationSetpoint() - arm.getWristCurrentRotationNativeUnits());

    boolean extensionIsFinished = extensionError < Constants.WRIST_ROTATION_IS_AT_SETPOINT_MARGIN_ENCODER_TICKS;

    /*
    SmartDashboard.putBoolean("EXT FIN", extensionIsFinished);
    SmartDashboard.putNumber("EXT error", extensionError);

    SmartDashboard.putBoolean("ROT FIN", rotationIsFinished);
    SmartDashboard.putNumber("ROT error", rotationError);
*/

    return extensionIsFinished;
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
