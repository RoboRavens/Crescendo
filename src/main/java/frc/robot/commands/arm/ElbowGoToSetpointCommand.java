// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.util.arm.LimbSetpoint;

public class ElbowGoToSetpointCommand extends Command {
  //private ArrayList<ArmSetpoint> subSetpoints = new ArrayList<ArmSetpoint>();
  private ElbowSubsystem elbow = Robot.ARM_SUBSYSTEM;
  private LimbSetpoint finalSetpoint;
  //private int setpointIterator = 0;
  private Timer timer = new Timer();
  private double timeoutSeconds = 0;

  /** Creates a new ArmGoToSetpointCommand. */
  public ElbowGoToSetpointCommand(LimbSetpoint setpoint) {
    this.finalSetpoint = setpoint;
    addRequirements(elbow);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elbow.setFinalArmRotationSetpoint(finalSetpoint.getRotationSetpoint());
    //arm.brakeDisable();

    timeoutSeconds = elbow.getCommandTimeoutSeconds();

    elbow.elbowMotionMagic();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    elbow.setElbowRotationPosition(finalSetpoint.getRotationSetpoint(), Constants.ELBOW_ROTATION_VELOCITY, Constants.ELBOW_ROTATION_ACCELERATION);
  }

  public boolean setpointIsFinished(LimbSetpoint setpoint) {
    double rotationError = Math.abs(setpoint.getRotationSetpoint() - elbow.getElbowCurrentRotationNativeUnits());

    boolean rotationIsFinished = rotationError < Constants.ELBOW_ROTATION_IS_AT_SETPOINT_MARGIN_ENCODER_TICKS;

    /*
    SmartDashboard.putBoolean("EXT FIN", extensionIsFinished);
    SmartDashboard.putNumber("EXT error", extensionError);

    SmartDashboard.putBoolean("ROT FIN", rotationIsFinished);
    SmartDashboard.putNumber("ROT error", rotationError);
*/

    return rotationIsFinished;
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
