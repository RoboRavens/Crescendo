// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.controls.ButtonCode;
import frc.controls.ButtonCode.Buttons;
import frc.robot.commands.LimbGoToSetpointCommand;
import frc.robot.commands.MoveElbowManuallyCommand;
import frc.robot.commands.drivetrain.DrivetrainAutoAimCommand;
import frc.robot.commands.drivetrain.DrivetrainDefaultCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.IntakeFeedCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.subsystems.AutoChooserSubsystemReact;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ReactDashSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TeleopDashboardSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.util.arm.LimbSetpoint;
import frc.util.StateManagement;
import frc.util.StateManagement.ArmUpTargetState;
import frc.util.StateManagement.ClimbPositionTargetState;
import frc.util.StateManagement.DrivetrainState;
import frc.util.StateManagement.IntakeTargetState;
import frc.util.StateManagement.LEDSignalTargetState;
import frc.util.StateManagement.LimelightDetectsNoteState;
import frc.util.StateManagement.LoadState;
import frc.util.StateManagement.OverallState;
import frc.util.StateManagement.ScoringTargetState;
import frc.util.StateManagement.ShooterRevTargetState;
import frc.util.StateManagement.TrapSourceLaneTargetState;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the manifest
 * file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  public static final LimelightHelpers LIMELIGHT_HELPERS = new LimelightHelpers();
  public static final LimelightSubsystem LIMELIGHT_SUBSYSTEM_ONE = new LimelightSubsystem("limelight-pick");
  public static final LimelightSubsystem LIMELIGHT_SUBSYSTEM_TWO = new LimelightSubsystem("limelight-front");
  public static final LimelightSubsystem LIMELIGHT_SUBSYSTEM_THREE = new LimelightSubsystem("limelight-backl");
  public static final LimelightSubsystem LIMELIGHT_SUBSYSTEM_FOUR = new LimelightSubsystem("limelight-backr");
  public static final DrivetrainSubsystem DRIVETRAIN_SUBSYSTEM = new DrivetrainSubsystem();
  public static final PoseEstimatorSubsystem POSE_ESTIMATOR_SUBSYSTEM = new PoseEstimatorSubsystem();
  public static final XboxController DRIVE_CONTROLLER = new XboxController(0);
  public static DriverStation.Alliance allianceColor = Alliance.Blue;
  public static final DrivetrainDefaultCommand DRIVETRAIN_DEFAULT_COMMAND = new DrivetrainDefaultCommand();
  public static final DrivetrainAutoAimCommand DRIVETRAIN_AUTO_AIM_COMMAND = new DrivetrainAutoAimCommand();
  public static final ReactDashSubsystem REACT_DASH_SUBSYSTEM = new ReactDashSubsystem();
  public static final AutoChooserSubsystemReact AUTO_CHOOSER = new AutoChooserSubsystemReact();
  public static final TeleopDashboardSubsystem TELEOP_DASHBOARD_SUBSYSTEM = new TeleopDashboardSubsystem();
  public static final ButtonCode BUTTON_CODE = new ButtonCode();
  public static final IntakeSubsystem INTAKE_SUBSYSTEM = new IntakeSubsystem();
  public static final ShooterSubsystem SHOOTER_SUBSYSTEM = new ShooterSubsystem();
  public static final ElbowSubsystem ELBOW_SUBSYSTEM = new ElbowSubsystem();
  public static final WristSubsystem WRIST_SUBSYSTEM = new WristSubsystem();
  // public static final XboxController XBOX_CONTROLLER = new XboxController(0);
  // States
  public static ScoringTargetState SCORING_TARGET_STATE = ScoringTargetState.SPEAKER;
  public static IntakeTargetState INTAKE_TARGET_STATE = IntakeTargetState.GROUND;
  public static LEDSignalTargetState LED_SIGNAL_TARGET_STATE = LEDSignalTargetState.NONE;
  public static TrapSourceLaneTargetState TRAP_SOURCE_LANE_TARGET_STATE = TrapSourceLaneTargetState.CENTER;
  public static ArmUpTargetState ARM_UP_TARGET_STATE = ArmUpTargetState.FREE;
  public static ShooterRevTargetState SHOOTER_REV_TARGET_STATE = ShooterRevTargetState.OFF;
  public static ClimbPositionTargetState CLIMB_POSITION_TARGET_STATE = ClimbPositionTargetState.LEFT_CLOSE;
  public static OverallState OVERALL_STATE = OverallState.EMPTY_TRANSIT;
  public static LoadState LOAD_STATE = LoadState.EMPTY;
  public static DrivetrainState DRIVETRAIN_STATE = DrivetrainState.FREEHAND;
  public static LimelightDetectsNoteState LIMELIGHT_DETECTS_NOTE_STATE = LimelightDetectsNoteState.NO_NOTE;

  @Override
  public void robotPeriodic() {
    SmartDashboard.putString("Alliance Color", allianceColor.name());
    CommandScheduler.getInstance().run();
    setDriverStationData();
    SmartDashboard.putString("Scoring Target State", SCORING_TARGET_STATE.toString());
    SmartDashboard.putString("Intake Target State", INTAKE_TARGET_STATE.toString());
    SmartDashboard.putString("LED Signal Target State", LED_SIGNAL_TARGET_STATE.toString());
    SmartDashboard.putString("Trap Source Lane Target State", TRAP_SOURCE_LANE_TARGET_STATE.toString());
    SmartDashboard.putString("Arm Up Target State", ARM_UP_TARGET_STATE.toString());
    SmartDashboard.putString("Shooter Rev Target State", SHOOTER_REV_TARGET_STATE.toString());
    SmartDashboard.putString("Climb Position Target State", CLIMB_POSITION_TARGET_STATE.toString());
    setNonButtonDependentOverallStates();
    // TODO: Create a method that returns the wrist setpoint, and replace the below
    // wrist rotation setpoints with that method
    LimbSetpoint.SPEAKER_SCORING = new LimbSetpoint("", 0, 0);
    LimbSetpoint.DEFENDED_SPEAKER_SCORING = new LimbSetpoint("", 0, 0);
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    DRIVETRAIN_SUBSYSTEM.setDefaultCommand(DRIVETRAIN_DEFAULT_COMMAND);
    
    new Trigger(() -> DRIVE_CONTROLLER.getLeftBumper()
        && (DRIVE_CONTROLLER.getRightBumper())
        && (DRIVE_CONTROLLER.getYButton()))
        .onTrue(new InstantCommand(() -> DRIVETRAIN_SUBSYSTEM.zeroGyroscope()));
    AUTO_CHOOSER.ShowTab();

    new Trigger(() -> SHOOTER_SUBSYSTEM.hasPiece() && DRIVE_CONTROLLER.getLeftBumper()).onTrue(new ShootCommand());
    
    
    new Trigger(() -> DRIVE_CONTROLLER.getLeftTriggerAxis() > .1 && Robot.LIMELIGHT_SUBSYSTEM_ONE.getTv() == 1).whileTrue(DRIVETRAIN_AUTO_AIM_COMMAND);

    configureButtonBindings();
    configureTriggers();
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    setDriverStationData();
    // PathPlannerTrajectory traj = exampleChoreoTraj.getTrajectory(new
    // ChassisSpeeds(0, 0, 0), new Rotation2d(0, 0));
    // DRIVETRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(traj).andThen(AutoBuilder.followPath(exampleChoreoTraj)).schedule();
    // PathPlannerTrajectory traj = sixNotePath.getTrajectory(new ChassisSpeeds(0,
    // 0, 0), new Rotation2d(0, 0));
    // DRIVETRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(traj).andThen(AutoBuilder.followPath(sixNotePath)).schedule();

    Command m_autonomousCommand = AUTO_CHOOSER.GetAutoCommand();

    // // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    setDriverStationData();
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  // Due to the manner in which the robot connects to the driver station,
  // which differs between the shop and match play,
  // this method needs to called both periodically AND in the auto/tele init
  // methods.
  private void setDriverStationData() {
    allianceColor = DriverStation.getAlliance().get();
    AUTO_CHOOSER.BuildAutoChooser(allianceColor);
  }

  private void configureButtonBindings() {
    // If the left trigger is held
    new Trigger(() -> DRIVE_CONTROLLER.getLeftTriggerAxis() > 0)
        .onTrue(new InstantCommand(() -> DRIVETRAIN_STATE = DrivetrainState.ROBOT_ALIGN))
        .onFalse(new InstantCommand(() -> DRIVETRAIN_STATE = DrivetrainState.FREEHAND));
    // If the robot is ready to shoot and we hold A, feed the note into the shooter
    new Trigger(() -> StateManagement.isRobotReadyToShoot() && DRIVE_CONTROLLER.getAButton())
        .onTrue(new IntakeFeedCommand(INTAKE_SUBSYSTEM));

    BUTTON_CODE.getButton(Buttons.GROUND_PICKUP_AND_SPEAKER_SCORING).and(() -> LOAD_STATE == LoadState.EMPTY)
        .whileTrue(new LimbGoToSetpointCommand(LimbSetpoint.GROUND_PICKUP));
    BUTTON_CODE.getButton(Buttons.GROUND_PICKUP_AND_SPEAKER_SCORING).and(() -> LOAD_STATE == LoadState.LOADED)
        .whileTrue(new LimbGoToSetpointCommand(LimbSetpoint.SPEAKER_SCORING));
    BUTTON_CODE.getButton(Buttons.DEFENDED_SPEAKER_SCORING)
        .whileTrue(new LimbGoToSetpointCommand(LimbSetpoint.DEFENDED_SPEAKER_SCORING));
    BUTTON_CODE.getButton(Buttons.AMP_SCORING)
        .whileTrue(new LimbGoToSetpointCommand(LimbSetpoint.AMP_SCORING));
    BUTTON_CODE.getButton(Buttons.TRAP_SCORING)
        .whileTrue(new LimbGoToSetpointCommand(LimbSetpoint.TRAP_SCORING));
    BUTTON_CODE.getButton(Buttons.AMP_AND_SPEAKER_SOURCE_INTAKE)
        .whileTrue(new LimbGoToSetpointCommand(LimbSetpoint.AMP_AND_SPEAKER_SOURCE_INTAKE));
    BUTTON_CODE.getButton(Buttons.TRAP_SOURCE_INTAKE)
        .whileTrue(new LimbGoToSetpointCommand(LimbSetpoint.TRAP_SOURCE_INTAKE));

    BUTTON_CODE.getButton(Buttons.MOVE_ELBOW_UP)
        .whileTrue(new MoveElbowManuallyCommand(Constants.MOVE_ELBOW_UP_MANUAL_POWER));
    BUTTON_CODE.getButton(Buttons.MOVE_ELBOW_DOWN)
        .whileTrue(new MoveElbowManuallyCommand(Constants.MOVE_ELBOW_DOWN_MANUAL_POWER));
    BUTTON_CODE.getButton(Buttons.MOVE_WRIST_UP)
        .whileTrue(new MoveElbowManuallyCommand(Constants.MOVE_WRIST_UP_MANUAL_POWER));
    BUTTON_CODE.getButton(Buttons.MOVE_WRIST_DOWN)
        .whileTrue(new MoveElbowManuallyCommand(Constants.MOVE_WRIST_DOWN_MANUAL_POWER));

  }

  private void setNonButtonDependentOverallStates() {
    // If the robot detects a note and has a target scoring state that requires a note
    if (OVERALL_STATE == OverallState.SEEKING_NOTE
        && LIMELIGHT_DETECTS_NOTE_STATE == LimelightDetectsNoteState.IN_RANGE) {
      OVERALL_STATE = OverallState.LOADING;
    }
    // If the robot does not have a note and is not in the process of loading a
    // note, but has a target scoring state that requires a note
    if (LOAD_STATE == LoadState.EMPTY && OVERALL_STATE != OverallState.LOADING
        && SCORING_TARGET_STATE != ScoringTargetState.CLIMB) {
      OVERALL_STATE = OverallState.SEEKING_NOTE;
    } else if (LOAD_STATE == LoadState.LOADED || LOAD_STATE == LoadState.TRAP_LOADED) {
      OVERALL_STATE = OverallState.LOADED_TRANSIT;
    }
  }

  private void configureTriggers() {
    // If we are seeking a note and intend to intake from the ground,
    // set our arm position to the ground setpoint
    new Trigger(() -> OVERALL_STATE == OverallState.SEEKING_NOTE
        && INTAKE_TARGET_STATE == IntakeTargetState.GROUND)
        .whileTrue(new LimbGoToSetpointCommand(LimbSetpoint.GROUND_PICKUP));
    // If we are seeking a note and intend to intake a regular note from the source,
    // set our arm position to the source setpoint
    new Trigger(() -> OVERALL_STATE == OverallState.SEEKING_NOTE
        && INTAKE_TARGET_STATE == IntakeTargetState.SOURCE)
        .whileTrue(new LimbGoToSetpointCommand(LimbSetpoint.AMP_AND_SPEAKER_SOURCE_INTAKE));
    // If we are seeking a note and intend to intake a note from the source to score
    // in the trap, set our arm position to the source trap setpoint
    new Trigger(() -> OVERALL_STATE == OverallState.SEEKING_NOTE
        && INTAKE_TARGET_STATE == IntakeTargetState.TRAP_SOURCE)
        .whileTrue(new LimbGoToSetpointCommand(LimbSetpoint.TRAP_SOURCE_INTAKE));
    // If the robot's overall state is loading the note, flash our
    // LEDs and start the intake
    new Trigger(() -> OVERALL_STATE == OverallState.LOADING)
        .whileTrue(new IntakeCommand(INTAKE_SUBSYSTEM)
            .alongWith(new InstantCommand(() -> System.out.println("Flash LEDs Green"))));
    // TODO:^^ implement a flash LEDs command ^^
    // If the robot is loaded with a note and we intend to score in the amp,
    // set our arm position to the amp setpoint
    new Trigger(() -> OVERALL_STATE == OverallState.LOADED_TRANSIT)
        .whileTrue(new LimbGoToSetpointCommand(LimbSetpoint.AMP_SCORING));
    // If the robot is loaded with a note and we intend to score in the trap,
    // set our arm position to the trap setpoint
    new Trigger(() -> OVERALL_STATE == OverallState.LOADED_TRANSIT)
        .whileTrue(new LimbGoToSetpointCommand(LimbSetpoint.TRAP_SCORING));
  }
}
