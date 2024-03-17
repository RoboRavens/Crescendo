// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compound;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.elbow.ElbowDefaultCommand;
import frc.robot.commands.elbow.ElbowGoToPositionCommand;
import frc.robot.commands.wrist.WristDefaultCommand;
import frc.robot.commands.wrist.WristGoToPositionCommand;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.util.Constants.ElbowConstants;
import frc.robot.util.Constants.WristConstants;
import frc.robot.util.arm.LimbSetpoint;

public class LimbGoToSetpointCommand extends Command {
  /** Creates a new LimbGoToSetpoint. */
  private LimbSetpoint _targetLimbSetPoint;
  private LimbGoToSetpointCommand(LimbSetpoint targetLimbSetPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    _targetLimbSetPoint = targetLimbSetPoint;
    addRequirements(Robot.ELBOW_SUBSYSTEM, Robot.WRIST_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("LimbGoToSetpointCommand: initialize");
    Robot.ELBOW_SUBSYSTEM.goToPosition(_targetLimbSetPoint.getElbowRotationPosition());
    Robot.WRIST_SUBSYSTEM.goToPosition(_targetLimbSetPoint.getWristRotationPosition());
    String name = _targetLimbSetPoint.getName();
    System.out.println("--- Limbsetpoint: " + name);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("LimbGoToSetpointCommand: end" + (interrupted ? " interrupted": ""));
    Robot.ELBOW_SUBSYSTEM.setPowerManually(0);
    Robot.WRIST_SUBSYSTEM.setPowerManually(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double elbowDiff = Math.abs(Robot.ELBOW_SUBSYSTEM.getPosition() - _targetLimbSetPoint.getElbowRotationPosition());
    double wristDiff = Math.abs(Robot.WRIST_SUBSYSTEM.getPosition() - _targetLimbSetPoint.getWristRotationPosition());
    if(elbowDiff <= ElbowConstants.IS_AT_SETPOINT_BUFFER && wristDiff <= WristConstants.IS_AT_SETPOINT_BUFFER){
      return true;
    }
    return false;
  }

  public static Command GetMoveSafelyCommand(LimbSetpoint targetLimbSetPoint){
    BooleanSupplier shouldRunAdvancedMovement = () -> {
      var elbowTargetDegrees = targetLimbSetPoint.getElbowRotationDegrees();
      var elbowCurrentDegrees = Math.toDegrees(Robot.ELBOW_SUBSYSTEM.getRadians());
      var wristTargetDegrees = targetLimbSetPoint.getWristRotationDegrees();
      var wristCurrentDegrees = Math.toDegrees(Robot.WRIST_SUBSYSTEM.getRadians());

      System.out.println("elbowTargetDegrees: " + elbowTargetDegrees);
      System.out.println("elbowCurrentDegrees: " + elbowCurrentDegrees);
      System.out.println("wristTargetDegrees: " + wristTargetDegrees);
      System.out.println("wristCurrentDegrees: " + wristCurrentDegrees);

      // going down through the threshold
      if (elbowCurrentDegrees > 0 && elbowTargetDegrees < 0) {
        return wristCurrentDegrees <= 0;
      }
      
      // going up through the threshold
      if (elbowCurrentDegrees < 0 && elbowTargetDegrees > 0 ) {
        return wristTargetDegrees <= 0;
      }
      
      return false;
    };

    var safeLimbSetpoint = new LimbSetpoint("Advanced Movement Safe Point", 20, 5);

    var advancedMovementCommand = new SequentialCommandGroup(
      new InstantCommand(() -> System.out.println("---Running advancedMovementCommand---")),
      //new ParallelDeadlineGroup(new WristGoToPositionCommand(WristSubsystem.getPositionFromRadians(Math.toRadians(5))), new ElbowDefaultCommand()),
      //new ParallelDeadlineGroup(new ElbowGoToPositionCommand(targetLimbSetPoint.getElbowRotationPosition()), new WristDefaultCommand()),
      new LimbGoToSetpointCommand(safeLimbSetpoint),
      new LimbGoToSetpointCommand(targetLimbSetPoint),
      new InstantCommand(() -> System.out.println("Finished advancedMovementCommand"))
    );

    var basicMovementCommand = new LimbGoToSetpointCommand(targetLimbSetPoint);

    return new ConditionalCommand(advancedMovementCommand, basicMovementCommand, shouldRunAdvancedMovement);
  }
}
