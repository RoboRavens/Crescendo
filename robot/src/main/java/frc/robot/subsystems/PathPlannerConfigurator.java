// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.commands.compound.LimbGoToSetpointCommand;
import frc.robot.commands.elbow.ElbowGoToPositionCommand;
import frc.robot.commands.intake.FeedWithSensorCommand;
import frc.robot.commands.intake.IntakeDefaultCommand;
import frc.robot.commands.intake.IntakeWithSensorAutoCommand;
import frc.robot.commands.intake.IntakeWithSensorCommand;
import frc.robot.commands.shooter.StartShooterCommand;
import frc.robot.commands.wrist.WristGoToPositionCommand;
import frc.robot.util.arm.LimbSetpoint;
import frc.robot.Robot;

public class PathPlannerConfigurator extends SubsystemBase {
  /** Creates a new PathPlannerConfigurator. */
  public PathPlannerConfigurator() {
    NamedCommands.registerCommand("StartShooterCommand", new StartShooterCommand());
    NamedCommands.registerCommand("IntakeNoteCommand", new IntakeWithSensorAutoCommand());
    NamedCommands.registerCommand("IntakeDefaultCommand", new IntakeDefaultCommand());
    NamedCommands.registerCommand("FeedNoteCommand", new FeedWithSensorCommand());
    NamedCommands.registerCommand("LimbGoToGroundSetpointCommand", LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.GROUND_PICKUP));
    NamedCommands.registerCommand("LimbGoToPreloadNoteScorePositionCommand", LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.SIX_NOTE_AUTO_PRELOAD_SCORING_SETPOINT));
    NamedCommands.registerCommand("LimbGoToGN1and2ScorePositionCommand", LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.SIX_NOTE_AUTO_GN_1_AND_2_SCORING_SETPOINT));
    NamedCommands.registerCommand("LimbGoToGN3ScorePositionCommand", LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.SIX_NOTE_AUTO_GN_3_SCORING_SETPOINT));
    NamedCommands.registerCommand("LimbGoToGN4And5ScorePositionCommand", LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.SIX_NOTE_AUTO_GN_4_and_5_SCORING_SETPOINT));
    NamedCommands.registerCommand("LimbStartConfigUpCommand", new ElbowGoToPositionCommand(LimbSetpoint.START_CONFIG_UP.getElbowRotationPosition())); // not a LimbSetpoint to optimize for time
    NamedCommands.registerCommand("LimbGoToArmUpShotCommand", LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.DEFENDED_SPEAKER_SCORING));
    NamedCommands.registerCommand("SouthCenterLimbGoToPreloadSetpointCommand", LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.SOUTH_CENTER_PRELOAD));
    NamedCommands.registerCommand("LimbGoToSourceSideAutoWingShot", LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.SOURCE_SIDE_AUTO_WING_SHOT));
    
    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
      Robot.DRIVETRAIN_SUBSYSTEM::getPose, // Robot pose supplier
      Robot.DRIVETRAIN_SUBSYSTEM::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      Robot.DRIVETRAIN_SUBSYSTEM::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      Robot.DRIVETRAIN_SUBSYSTEM::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
              DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, // Max module speed, in m/s
              0.37, // Drive base radius in meters. Distance from robot center to furthest module.
              new ReplanningConfig() // Default path replanning config. See the API for the options here
      ),
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      Robot.DRIVETRAIN_SUBSYSTEM // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
