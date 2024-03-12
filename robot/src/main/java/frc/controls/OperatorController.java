// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.controls;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.compound.LimbGoToSetpointCommand;
import frc.robot.commands.elbow.ElbowIncrementPositionCommand;
import frc.robot.commands.elbow.ElbowDecrementPositionCommand;
import frc.robot.commands.elbow.ElbowMoveWithJoystickCommand;
import frc.robot.commands.elbow.ElbowSuspendLimitsCommand;
import frc.robot.commands.wrist.WristMoveWithJoystickCommand;
import frc.robot.commands.wrist.WristSuspendLimitsCommand;
import frc.robot.commands.shooter.StartShooterCommand;
import frc.robot.util.arm.LimbSetpoint;

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
        .onTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.DEFENDED_SPEAKER_SCORING));
        _operatorController.x()
        .onTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.AMP_SCORING));
        _operatorController.y()
        .onTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.SOURCE_INTAKE));
        _operatorController.a()
        .onTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.GROUND_PICKUP));
        _operatorController.leftTrigger().and(_operatorController.rightBumper())
        .onTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.START_CONFIG_UP));

        _operatorController.leftBumper().whileTrue(new StartShooterCommand());

        _operatorController.leftTrigger().and(() -> Math.abs(_operatorController.getLeftY()) > .1).whileTrue(new ElbowMoveWithJoystickCommand(_operatorController));
        _operatorController.leftTrigger().and(() -> Math.abs(_operatorController.getRightY()) > .1).whileTrue(new WristMoveWithJoystickCommand(_operatorController));
   
        _operatorController.leftTrigger().and(_operatorController.povUp()).onTrue(new ElbowIncrementPositionCommand());
        _operatorController.leftTrigger().and(_operatorController.povDown()).onTrue(new ElbowDecrementPositionCommand());
        
        _operatorController.leftTrigger().and(_operatorController.rightTrigger()).and(_operatorController.start()).whileTrue(new ElbowSuspendLimitsCommand());
        _operatorController.leftTrigger().and(_operatorController.rightTrigger()).and(_operatorController.back()).whileTrue(new WristSuspendLimitsCommand());
    }      
}
