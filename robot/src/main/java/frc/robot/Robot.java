// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.controls.ButtonCode;
import frc.controls.ButtonCode.Buttons;
import frc.controls.ButtonCode.Toggle;
import frc.controls.OperatorController;
import frc.robot.commands.compound.LimbGoToSetpointCommand;
import frc.robot.commands.drivetrain.DriveTwoInchesCommand;
import frc.robot.commands.drivetrain.DrivetrainAutoAimNoteCommand;
import frc.robot.commands.drivetrain.DrivetrainDefaultCommand;
import frc.robot.commands.elbow.ElbowDefaultCommand;
import frc.robot.commands.elbow.ElbowMoveManuallyCommand;
import frc.robot.commands.elbow.ElbowOffsetCommand;
import frc.robot.commands.intake.FeedWithSensorCommand;
import frc.robot.commands.intake.IntakeDefaultCommand;
import frc.robot.commands.intake.IntakeReverseCommand;
import frc.robot.commands.intake.IntakeWithSensorTeleopCommand;
import frc.robot.commands.shooter.ShooterReverseCommand;
import frc.robot.commands.shooter.StartShooterCommand;
import frc.robot.commands.wrist.WristDefaultCommand;
import frc.robot.commands.wrist.WristGoToPositionCommand;
import frc.robot.commands.wrist.WristGoToSpeakerAngleCommand;
import frc.robot.commands.wrist.WristMoveManuallyCommand;
import frc.robot.commands.wrist.WristOffsetCommand;
import frc.robot.commands.wrist.WristSetPowerCommand;
import frc.robot.subsystems.AutoChooserSubsystemReact;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDsSubsystem24;
import frc.robot.subsystems.LimelightPickupSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PathPlannerConfigurator;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ReactDashSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TeleopDashboardSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.LEDsSubsystem24.LEDsPattern;
import frc.robot.util.Constants.Constants;
import frc.robot.util.Constants.WristConstants;
import frc.robot.util.arm.LimbSetpoint;
import frc.util.StateManagement.ArmUpTargetState;
import frc.util.StateManagement.ClimbPositionTargetState;
import frc.util.StateManagement.DrivetrainState;
import frc.util.StateManagement.IntakeTargetState;
import frc.util.StateManagement.LEDSignalTargetState;
import frc.util.StateManagement.LimelightDetectsNoteState;
import frc.util.StateManagement.LimelightOverrideState;
import frc.util.StateManagement.LoadState;
import frc.util.StateManagement.OverallState;
import frc.util.StateManagement.ScoringTargetState;
import frc.util.StateManagement.SelectedShotTargetState;
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
  public static final LimelightPickupSubsystem LIMELIGHT_PICKUP = new LimelightPickupSubsystem("limelight-pickup");
  public static final LimelightSubsystem LIMELIGHT_LEFT = new LimelightSubsystem("limelight-left");
  public static final LimelightSubsystem LIMELIGHT_RIGHT = new LimelightSubsystem("limelight-right");
  public static final LimelightSubsystem LIMELIGHT_BACK = new LimelightSubsystem("limelight-back");
  public static final DrivetrainSubsystem DRIVETRAIN_SUBSYSTEM = new DrivetrainSubsystem();
  public static final PoseEstimatorSubsystem POSE_ESTIMATOR_SUBSYSTEM = new PoseEstimatorSubsystem();
  public static final LimelightHelpers LIMELIGHT_HELPERS = new LimelightHelpers();

  public static final CommandXboxController COMMAND_DRIVE_CONTROLLER = new CommandXboxController(RobotMap.DRIVE_CONTROLLER_PORT);
  public static final XboxController DRIVE_CONTROLLER = COMMAND_DRIVE_CONTROLLER.getHID();
  public static DriverStation.Alliance allianceColor = Alliance.Blue;
  public static final DrivetrainAutoAimNoteCommand DRIVETRAIN_AUTO_AIM_NOTE_COMMAND = new DrivetrainAutoAimNoteCommand();
  public static final ReactDashSubsystem REACT_DASH_SUBSYSTEM = new ReactDashSubsystem();
  public static final AutoChooserSubsystemReact AUTO_CHOOSER = new AutoChooserSubsystemReact();
  public static final TeleopDashboardSubsystem TELEOP_DASHBOARD_SUBSYSTEM = new TeleopDashboardSubsystem();
  public static final ButtonCode BUTTON_CODE = new ButtonCode();
  public static final IntakeSubsystem INTAKE_SUBSYSTEM = new IntakeSubsystem();
  public static final ShooterSubsystem SHOOTER_SUBSYSTEM = new ShooterSubsystem();
  public static final ElbowSubsystem ELBOW_SUBSYSTEM = new ElbowSubsystem();
  public static final WristSubsystem WRIST_SUBSYSTEM = new WristSubsystem();
  public static final LEDsSubsystem24 LED_SUBSYSTEM = new LEDsSubsystem24();
  public static final PathPlannerConfigurator PATH_PLANNER_CONFIGURATOR = new PathPlannerConfigurator();
  public static final IntakeDefaultCommand INTAKE_DEFAULT_COMMAND = new IntakeDefaultCommand();
  public static final WristSetPowerCommand WRIST_SET_POWER_COMMAND = new WristSetPowerCommand();
  public static final ClimberSubsystem LEFT_CLIMBER_SUBSYSTEM = new ClimberSubsystem(RobotMap.LEFT_CLIMBER_MOTOR, true);
  public static final ClimberSubsystem RIGHT_CLIMBER_SUBSYSTEM = new ClimberSubsystem(RobotMap.RIGHT_CLIMBER_MOTOR, false);

  // DEFAULT COMMANDS
  public static final DrivetrainDefaultCommand DRIVETRAIN_DEFAULT_COMMAND = new DrivetrainDefaultCommand();
  public static final ElbowDefaultCommand ELBOW_DEFAULT_COMMAND = new ElbowDefaultCommand();
  public static final WristDefaultCommand WRIST_DEFAULT_COMMAND = new WristDefaultCommand();
  
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
  public static boolean cutPower = false;
  public static SelectedShotTargetState SELECTED_SHOT_TARGET_STATE = SelectedShotTargetState.SUBWOOFER_SHOT;
  public static LimelightOverrideState LIMELIGHT_OVERRIDE_STATE = LimelightOverrideState.OVERRIDE_OFF;
  public static boolean autoRotationAlignEnabled = false;

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
    SmartDashboard.putNumber("Distance from Speaker", Robot.DRIVETRAIN_SUBSYSTEM.getDistanceFromSpeaker());
    SmartDashboard.putString("Limelight Override State", LIMELIGHT_OVERRIDE_STATE.toString());
    SmartDashboard.putString("Shot Selection Target State", SELECTED_SHOT_TARGET_STATE.toString());
    setNonButtonDependentOverallStates();

    SmartDashboard.putData("Drivetrain Command", DRIVETRAIN_SUBSYSTEM);
    SmartDashboard.putData("Wrist Command", WRIST_SUBSYSTEM);
    SmartDashboard.putData("Arm Command", ELBOW_SUBSYSTEM);
    SmartDashboard.putData("Intake Command", INTAKE_SUBSYSTEM);
    SmartDashboard.putData("Shooter Command", SHOOTER_SUBSYSTEM);
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    DRIVETRAIN_SUBSYSTEM.setDefaultCommand(DRIVETRAIN_DEFAULT_COMMAND);
    ELBOW_SUBSYSTEM.setDefaultCommand(ELBOW_DEFAULT_COMMAND);
    WRIST_SUBSYSTEM.setDefaultCommand(WRIST_DEFAULT_COMMAND);
    INTAKE_SUBSYSTEM.setDefaultCommand(INTAKE_DEFAULT_COMMAND);

    SmartDashboard.putData(ELBOW_SUBSYSTEM);
    SmartDashboard.putData(WRIST_SUBSYSTEM);

    AUTO_CHOOSER.ShowTab();
    
    new Trigger(() -> DRIVE_CONTROLLER.getLeftTriggerAxis() > .1
      && Robot.LIMELIGHT_PICKUP.hasVisionTargetBuffered()
      && INTAKE_TARGET_STATE == IntakeTargetState.GROUND
      && Robot.INTAKE_SUBSYSTEM.driverCanIntake()
      && Robot.INTAKE_SUBSYSTEM.hasPieceAnywhere() == false)
    .whileTrue(DRIVETRAIN_AUTO_AIM_NOTE_COMMAND.alongWith(new IntakeWithSensorTeleopCommand()));

    // The autoRotationAlignEnabled boolean is used in the drivetrain subsystem to enable rotation align without restricting x and y control
    new Trigger(() -> DRIVE_CONTROLLER.getLeftTriggerAxis() > .1
      && Robot.INTAKE_SUBSYSTEM.hasPieceAnywhere())
    .onTrue(new InstantCommand(() -> autoRotationAlignEnabled = true))
    .onFalse(new InstantCommand(() -> autoRotationAlignEnabled = false));

    configureDriveControllerBindings();
    // configureAutomatedBehaviorBindings();
    configureButtonBindings();
    configureOverrideBindings();
    OperatorController.enable();
  }
  
  private void configureDriveControllerBindings() {
    // If the left trigger is held
    new Trigger(() -> DRIVE_CONTROLLER.getLeftTriggerAxis() > 0)
        .onTrue(new InstantCommand(() -> DRIVETRAIN_STATE = DrivetrainState.ROBOT_ALIGN))
        .onFalse(new InstantCommand(() -> DRIVETRAIN_STATE = DrivetrainState.FREEHAND));
    
    // new Trigger(() -> DRIVE_CONTROLLER.getLeftBumper()
    //     && (DRIVE_CONTROLLER.getRightBumper())
    //     && (DRIVE_CONTROLLER.getYButton()))
    //     .onTrue(new InstantCommand(() -> DRIVETRAIN_SUBSYSTEM.zeroGyroscope()));
    // If the robot is ready to shoot and we hold A, feed the note into the shooter
    //new Trigger(() -> StateManagement.isRobotReadyToShoot() && DRIVE_CONTROLLER.getAButton())
    //  .onTrue(new IntakeFeedCommand(INTAKE_SUBSYSTEM));

    COMMAND_DRIVE_CONTROLLER.leftBumper()
      .and(new Trigger(() -> Robot.INTAKE_SUBSYSTEM.driverCanIntake()))
      .whileTrue(new IntakeWithSensorTeleopCommand());
    COMMAND_DRIVE_CONTROLLER.rightBumper().whileTrue(new FeedWithSensorCommand());

    new Trigger(() -> SHOOTER_SUBSYSTEM.hasPiece()).onTrue(
      new InstantCommand(() -> {
        System.out.println("rumble");
        DRIVE_CONTROLLER.setRumble(RumbleType.kLeftRumble, 0.5);
      })
      .andThen(new WaitCommand(0.25))
      .andThen(new InstantCommand(() -> DRIVE_CONTROLLER.setRumble(RumbleType.kLeftRumble, 0)))
    );

    new Trigger(() -> DRIVE_CONTROLLER.getRightTriggerAxis() > 0.1)
      .onTrue(new InstantCommand(() -> cutPower = true))
      .onFalse(new InstantCommand(() -> cutPower = false));

    new Trigger(() -> DRIVE_CONTROLLER.getXButton())
      .whileTrue(new IntakeReverseCommand());

    new Trigger(() -> DRIVE_CONTROLLER.getXButton() 
      && SHOOTER_SUBSYSTEM.getLeftRpm() < 5 
      && SHOOTER_SUBSYSTEM.getRightRpm() < 5)
      .whileTrue(new ShooterReverseCommand());

    COMMAND_DRIVE_CONTROLLER.povRight().toggleOnTrue(new DriveTwoInchesCommand('R'));
    COMMAND_DRIVE_CONTROLLER.povUp().toggleOnTrue(new DriveTwoInchesCommand('F'));
    COMMAND_DRIVE_CONTROLLER.povDown().toggleOnTrue(new DriveTwoInchesCommand('B'));
    COMMAND_DRIVE_CONTROLLER.povLeft().toggleOnTrue(new DriveTwoInchesCommand('L'));
    
    new Trigger(() -> DRIVE_CONTROLLER.getYButton())
      .onTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.AMP_SCORING));
    
    new Trigger(() -> DRIVE_CONTROLLER.getBButton() && (SHOOTER_SUBSYSTEM.hasPiece() || INTAKE_TARGET_STATE == IntakeTargetState.GROUND))
      .onTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.DEFENDED_SPEAKER_SCORING));

    new Trigger(() -> DRIVE_CONTROLLER.getBButton() && (SHOOTER_SUBSYSTEM.hasPiece() == false && INTAKE_TARGET_STATE == IntakeTargetState.SOURCE))
      .onTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.SOURCE_INTAKE));
    
    new Trigger(() -> DRIVE_CONTROLLER.getBButton() == false && ARM_UP_TARGET_STATE == ArmUpTargetState.FREE)
      .onTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.GROUND_PICKUP));

    new Trigger(() -> DRIVE_CONTROLLER.getStartButton() && DRIVE_CONTROLLER.getBackButton())
        .onTrue(new InstantCommand(() -> DRIVETRAIN_SUBSYSTEM.zeroGyroscope()));
	}

  private void configureAutomatedBehaviorBindings() {
    new Trigger(() -> SHOOTER_REV_TARGET_STATE == ShooterRevTargetState.ON && DriverStation.isTeleop())
      .whileTrue(new StartShooterCommand());

    new Trigger(() -> ARM_UP_TARGET_STATE == ArmUpTargetState.UP && DriverStation.isTeleop())
      .whileTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.DEFENDED_SPEAKER_SCORING));

    new Trigger(() -> LIMELIGHT_OVERRIDE_STATE == LimelightOverrideState.OVERRIDE_ON && SELECTED_SHOT_TARGET_STATE == SelectedShotTargetState.SUBWOOFER_SHOT && DriverStation.isTeleop())
      .onTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.SPEAKER_SCORING));

    new Trigger(() -> LIMELIGHT_OVERRIDE_STATE == LimelightOverrideState.OVERRIDE_ON && SELECTED_SHOT_TARGET_STATE == SelectedShotTargetState.STARTING_LINE_SHOT && DriverStation.isTeleop())
      .onTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.STARTING_LINE_SCORING));

    new Trigger(() -> LIMELIGHT_OVERRIDE_STATE == LimelightOverrideState.OVERRIDE_ON && SELECTED_SHOT_TARGET_STATE == SelectedShotTargetState.PODIUM_SHOT && DriverStation.isTeleop())
      .onTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.PODIUM_SCORING));

    new Trigger(() -> ARM_UP_TARGET_STATE == ArmUpTargetState.FREE && Robot.INTAKE_SUBSYSTEM.hasPieceAnywhere() == false && DriverStation.isTeleop())
      .onTrue(new WristGoToPositionCommand(WristConstants.DEGREES_FLOOR_PICKUP));
  }

	/** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    LED_SUBSYSTEM.setPattern(LEDsPattern.Solid);
    LED_SUBSYSTEM.setColor(Color.kBlack);
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
    SmartDashboard.putBoolean("auto rotation align enabled", autoRotationAlignEnabled);
    this.runLedLogic();
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
    LED_SUBSYSTEM.setPattern(LEDsPattern.Solid);
    LED_SUBSYSTEM.setColor(Color.kBlack);
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
    allianceColor = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue);
    AUTO_CHOOSER.BuildAutoChooser(allianceColor);
  }

  private void runLedLogic() {
    boolean shooterAtSpeed = SHOOTER_SUBSYSTEM.shooterUpToSpeed();
    boolean hasPieceAnywhere = INTAKE_SUBSYSTEM.hasPieceAnywhere();
    boolean aimedAtGoal = DRIVETRAIN_SUBSYSTEM.getIsRobotRotationInSpeakerRange();
    boolean posessesIndexedPiece = INTAKE_SUBSYSTEM.getPossesesIndexedPiece();
    boolean robotSeesNote = Robot.LIMELIGHT_PICKUP.hasVisionTargetBuffered()
      && INTAKE_TARGET_STATE == IntakeTargetState.GROUND
      && Robot.INTAKE_SUBSYSTEM.driverCanIntake();

    if (shooterAtSpeed) {
      LED_SUBSYSTEM.setPattern(LEDsPattern.Blinking);
    } else {
      LED_SUBSYSTEM.setPattern(LEDsPattern.Solid);
    }

    if (shooterAtSpeed && !hasPieceAnywhere) {
      LED_SUBSYSTEM.setColor(Color.kYellow);
    } else if (robotSeesNote) {
      LED_SUBSYSTEM.setColor(Color.kOrange);
    } else if (posessesIndexedPiece) {
      if (aimedAtGoal) {
        LED_SUBSYSTEM.setColor(Color.kGreen);
      } else {
        LED_SUBSYSTEM.setColor(Color.kBlue);
      }
    } else if (hasPieceAnywhere) {
      LED_SUBSYSTEM.setColor(Color.kPurple);
    } else {
      LED_SUBSYSTEM.setColor(Color.kDarkSlateGray);
    }
  }

  private void configureButtonBindings() {
    // BUTTON_CODE.getButton(Buttons.GROUND_PICKUP_AND_SPEAKER_SCORING).and(() -> LOAD_STATE == LoadState.EMPTY)
    //     .onTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.GROUND_PICKUP));
    // BUTTON_CODE.getButton(Buttons.GROUND_PICKUP_AND_SPEAKER_SCORING).and(() -> LOAD_STATE == LoadState.LOADED)
    //     .onTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.SPEAKER_SCORING));
    BUTTON_CODE.getButton(Buttons.DEFENDED_SPEAKER_SCORING)
      .onTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.DEFENDED_SPEAKER_SCORING));
    BUTTON_CODE.getButton(Buttons.AMP_SCORING)
      .onTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.AMP_SCORING));
    // BUTTON_CODE.getButton(Buttons.TRAP_SCORING)
    //     .onTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.TRAP_SCORING));
    BUTTON_CODE.getButton(Buttons.SOURCE_INTAKE)
      .onTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.SOURCE_INTAKE));
    // BUTTON_CODE.getButton(Buttons.TRAP_SOURCE_INTAKE)
    //     .onTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.TRAP_SOURCE_INTAKE));
    BUTTON_CODE.getButton(Buttons.GROUND_PICKUP_AND_SPEAKER_SCORING)
      .onTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.GROUND_PICKUP));

    BUTTON_CODE.getButton(Buttons.ARM_RELEASE_SNAPPER)
      .onTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.START_CONFIG_UP));

    BooleanSupplier manualOverride = () -> BUTTON_CODE.getSwitch(Toggle.MOVE_WITH_MANUAL_POWER).getAsBoolean() == false;
    var elbowUpCommand = new ConditionalCommand(new ElbowMoveManuallyCommand(Constants.MOVE_ELBOW_UP_MANUAL_POWER), new ElbowOffsetCommand(0.5), manualOverride);
    var elbowDownCommand = new ConditionalCommand(new ElbowMoveManuallyCommand(Constants.MOVE_ELBOW_DOWN_MANUAL_POWER), new ElbowOffsetCommand(-0.5), manualOverride);
    var wristUpCommand = new ConditionalCommand(new WristMoveManuallyCommand(Constants.MOVE_WRIST_UP_MANUAL_POWER), new WristOffsetCommand(0.5), manualOverride);
    var wristDownCommand = new ConditionalCommand(new WristMoveManuallyCommand(Constants.MOVE_WRIST_DOWN_MANUAL_POWER), new WristOffsetCommand(-0.5), manualOverride);

    BUTTON_CODE.getButton(Buttons.MOVE_ELBOW_UP).and(BUTTON_CODE.getSwitch(Toggle.MOVE_WITH_MANUAL_POWER))
        .whileTrue(elbowUpCommand);
    BUTTON_CODE.getButton(Buttons.MOVE_ELBOW_DOWN)
        .whileTrue(elbowDownCommand);
    BUTTON_CODE.getButton(Buttons.MOVE_WRIST_UP)
        .whileTrue(wristUpCommand);
    BUTTON_CODE.getButton(Buttons.MOVE_WRIST_DOWN)
        .whileTrue(wristDownCommand);

    BUTTON_CODE.getButton(Buttons.SHOOTER_REV)
      .whileTrue(new InstantCommand(() -> SHOOTER_REV_TARGET_STATE = ShooterRevTargetState.ON))
      .onFalse(new InstantCommand(() -> SHOOTER_REV_TARGET_STATE = ShooterRevTargetState.OFF));

    BUTTON_CODE.getButton(Buttons.SHOOTER_REV)
      .whileTrue(new StartShooterCommand());

    BUTTON_CODE.getButton(Buttons.SHOOTER_REV)
      .onTrue(new InstantCommand(() -> SHOOTER_REV_TARGET_STATE = ShooterRevTargetState.ON));

    BUTTON_CODE.getSwitch(Toggle.SHOOTER_ANGLE_FROM_DISTANCE)
      .whileTrue(new WristGoToSpeakerAngleCommand());

    BUTTON_CODE.getButton(Buttons.SPEAKER_CLOSE_SHOT)
      .whileTrue(new InstantCommand(() -> 
      {
        double shooterAngleRadians = Math.toRadians(Robot.SHOOTER_SUBSYSTEM.getShooterAngleMapDown(0.9144));
        WRIST_SUBSYSTEM.setTargetPosition(WristSubsystem.getPositionFromRadians(shooterAngleRadians));
      }));

    // BUTTON_CODE.getButton(Buttons.SPEAKER_MID_SHOT)
    //   .whileTrue(new InstantCommand(() -> 
    //   {
    //     double shooterAngleRadians = Math.toRadians(Robot.SHOOTER_SUBSYSTEM.getShooterAngleMapDown(2.032));
    //     WRIST_SUBSYSTEM.setTargetPosition(WristSubsystem.getPositionFromRadians(shooterAngleRadians));
    //   }));

    BUTTON_CODE.getButton(Buttons.SPEAKER_FAR_SHOT)
      .whileTrue(new InstantCommand(() -> 
      {
        double shooterAngleRadians = Math.toRadians(Robot.SHOOTER_SUBSYSTEM.getShooterAngleMapDown(2.3368));
        WRIST_SUBSYSTEM.setTargetPosition(WristSubsystem.getPositionFromRadians(shooterAngleRadians));
      }));
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    System.out.println("disabled init");
  }

  @Override
  public void disabledPeriodic() {
    LED_SUBSYSTEM.setPattern(LEDsPattern.Rainbow);
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

  private void configureOverrideBindings() {
    // // If the arm-up toggle is on and our intake target is ground,
    // // set the intake target to the source
    // new Trigger(() -> ARM_UP_TARGET_STATE == ArmUpTargetState.UP
    //     && INTAKE_TARGET_STATE == IntakeTargetState.GROUND)
    //     .whileTrue(new InstantCommand(() -> INTAKE_TARGET_STATE = IntakeTargetState.SOURCE));
    // // If we are seeking a note and intend to intake from the ground,
    // // set our arm position to the ground setpoint
    // new Trigger(() -> OVERALL_STATE == OverallState.SEEKING_NOTE
    //     && INTAKE_TARGET_STATE == IntakeTargetState.GROUND)
    //     .whileTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.GROUND_PICKUP));
    // // If we are seeking a note and intend to intake a regular note from the source,
    // // set our arm position to the source setpoint
    // new Trigger(() -> OVERALL_STATE == OverallState.SEEKING_NOTE
    //     && INTAKE_TARGET_STATE == IntakeTargetState.SOURCE)
    //     .whileTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.AMP_AND_SPEAKER_SOURCE_INTAKE));
    // // If we are seeking a note and intend to intake a note from the source to score
    // // in the trap, set our arm position to the source trap setpoint
    // new Trigger(() -> OVERALL_STATE == OverallState.SEEKING_NOTE
    //     && INTAKE_TARGET_STATE == IntakeTargetState.TRAP_SOURCE)
    //     .whileTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.TRAP_SOURCE_INTAKE));
    // // If the robot's overall state is loading the note, flash our
    // // LEDs and start the intake
    // new Trigger(() -> OVERALL_STATE == OverallState.LOADING)
    //     .whileTrue(new IntakeCommand(INTAKE_SUBSYSTEM)
    //         .alongWith(new InstantCommand(() -> System.out.println("Flash LEDs Green"))));
    // // TODO:^^ implement a flash LEDs command ^^
    // // If the robot is loaded with a note and we intend to score in the amp,
    // // set our arm position to the amp setpoint
    // new Trigger(() -> OVERALL_STATE == OverallState.LOADED_TRANSIT 
    //     && SCORING_TARGET_STATE == ScoringTargetState.AMP)
    //     .whileTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.AMP_SCORING));
    // // If the robot is loaded with a note and we intend to score in the trap,
    // // set our arm position to the trap setpoint
    // new Trigger(() -> OVERALL_STATE == OverallState.LOADED_TRANSIT
    //     && SCORING_TARGET_STATE == ScoringTargetState.TRAP)
    //     .whileTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.TRAP_SCORING));
    // // If the robot is loaded with a note and we intend to score in the speaker,
    // // set our arm position to the speaker setpoint
    // new Trigger(() -> OVERALL_STATE == OverallState.LOADED_TRANSIT
    //     && SCORING_TARGET_STATE == ScoringTargetState.SPEAKER
    //     && ARM_UP_TARGET_STATE == ArmUpTargetState.FREE)
    //     .whileTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.SPEAKER_SCORING));
    // // Otherwise, if the arm-up toggle is on,
    // // set our arm position to the "up configuration"
    // new Trigger(() -> OVERALL_STATE == OverallState.LOADED_TRANSIT
    //     && SCORING_TARGET_STATE == ScoringTargetState.SPEAKER
    //     && ARM_UP_TARGET_STATE == ArmUpTargetState.UP)
    //     .whileTrue(LimbGoToSetpointCommand.GetMoveSafelyCommand(LimbSetpoint.SPEAKER_SCORING_ARM_UP));
  }
}
