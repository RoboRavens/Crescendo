// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.controls;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.climber.SetClimberToPowerCommand;
import frc.robot.commands.compound.LimbGoToSetpointCommand;
import frc.robot.commands.elbow.ElbowDecrementPositionCommand;
import frc.robot.commands.elbow.ElbowIncrementPositionCommand;
import frc.robot.commands.elbow.ElbowMoveWithJoystickCommand;
import frc.robot.commands.elbow.ElbowSuspendLimitsCommand;
import frc.robot.commands.shooter.ShooterTestingCommand;
import frc.robot.commands.shooter.StartShooterCommand;
import frc.robot.commands.wrist.WristDecrementPositionCommand;
import frc.robot.commands.wrist.WristIncrementPositionCommand;
import frc.robot.commands.wrist.WristMoveWithJoystickCommand;
import frc.robot.commands.wrist.WristSuspendLimitsCommand;
import frc.robot.util.arm.LimbSetpoint;
import frc.util.StateManagement.SelectedShotTargetState;
import frc.util.StateManagement.ShooterRevTargetState;

/** Add your docs here. */
public class OperatorController {
    private static CommandXboxController _operatorController;

    public static void enable() {
        _operatorController = new CommandXboxController(RobotMap.OPERATOR_CONTROLLER_PORT);

        _operatorController.start().and(_operatorController.back())
            .onTrue(new InstantCommand(() -> {
                System.out.println("OPERATOR CONTROLLER: RESET ARM POSITION TO GROUND PICKUP");
                Robot.WRIST_SUBSYSTEM.resetPosition();
                Robot.ELBOW_SUBSYSTEM.resetPosition();
            }).ignoringDisable(true));

        _operatorController.b()
        .onTrue(new InstantCommand(() -> Robot.SELECTED_SHOT_TARGET_STATE = SelectedShotTargetState.STARTING_LINE_SHOT));
        /*_operatorController.x()
        .onTrue(new InstantCommand(
            () -> Robot.LIMELIGHT_OVERRIDE_STATE = 
            Robot.LIMELIGHT_OVERRIDE_STATE == LimelightOverrideState.OVERRIDE_ON 
                ? LimelightOverrideState.OVERRIDE_OFF
                : LimelightOverrideState.OVERRIDE_ON
        ));*/
        _operatorController.x().onTrue(new InstantCommand(() -> Robot.SHOOTER_SUBSYSTEM.toggleShooterSpeeds()));
        _operatorController.y()
        .onTrue(new InstantCommand(() -> Robot.SELECTED_SHOT_TARGET_STATE = SelectedShotTargetState.PODIUM_SHOT));
        _operatorController.a()
        .onTrue(new InstantCommand(() -> Robot.SELECTED_SHOT_TARGET_STATE = SelectedShotTargetState.SUBWOOFER_SHOT));
        
        _operatorController.leftTrigger().and(_operatorController.rightBumper())
        .onTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.START_CONFIG_UP));

        _operatorController.leftBumper().whileTrue(new StartShooterCommand());
        _operatorController.leftBumper().whileTrue(new InstantCommand(() -> Robot.SHOOTER_REV_TARGET_STATE = ShooterRevTargetState.ON)).onFalse(new InstantCommand(() -> Robot.SHOOTER_REV_TARGET_STATE = ShooterRevTargetState.OFF));

        _operatorController.leftTrigger().and(() -> Math.abs(_operatorController.getLeftY()) > .1).whileTrue(new ElbowMoveWithJoystickCommand(_operatorController));
        _operatorController.leftTrigger().and(() -> Math.abs(_operatorController.getRightY()) > .1).whileTrue(new WristMoveWithJoystickCommand(_operatorController));
   
        _operatorController.leftTrigger().and(_operatorController.povUp()).onTrue(new ElbowIncrementPositionCommand());
        _operatorController.leftTrigger().and(_operatorController.povDown()).onTrue(new ElbowDecrementPositionCommand());
        _operatorController.leftTrigger().and(_operatorController.povRight()).onTrue(new WristIncrementPositionCommand());
        _operatorController.leftTrigger().and(_operatorController.povLeft()).onTrue(new WristDecrementPositionCommand());
        _operatorController.leftTrigger().negate().and(_operatorController.povDown()).onTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.SPEAKER_SCORING));
        _operatorController.leftTrigger().negate().and(_operatorController.povUp()).whileTrue(new ShooterTestingCommand());

        _operatorController.leftTrigger().and(_operatorController.rightTrigger()).and(_operatorController.start()).whileTrue(new ElbowSuspendLimitsCommand());
        _operatorController.leftTrigger().and(_operatorController.rightTrigger()).and(_operatorController.back()).whileTrue(new WristSuspendLimitsCommand());

        _operatorController.leftTrigger().negate().and(() -> Math.abs(_operatorController.getLeftY()) > .1)
        .whileTrue(new SetClimberToPowerCommand(() -> _operatorController.getLeftY() * -1, Robot.LEFT_CLIMBER_SUBSYSTEM));
        _operatorController.leftTrigger().negate().and(() -> Math.abs(_operatorController.getRightY()) > .1)
        .whileTrue(new SetClimberToPowerCommand(() -> _operatorController.getRightY() * -1, Robot.RIGHT_CLIMBER_SUBSYSTEM));
    }
}
