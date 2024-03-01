// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.controls.ButtonCode;
import frc.controls.ButtonCode.Buttons;
import frc.robot.commands.drivetrain.DrivetrainDefaultCommand;
import frc.robot.commands.elbow.ElbowDefaultCommand;
import frc.robot.commands.elbow.ElbowGoToPositionCommand;
import frc.robot.commands.intake.IntakeCommand;
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
import frc.util.StateManagement.ArmUpTargetState;
import frc.util.StateManagement.ClimbPositionTargetState;
import frc.util.StateManagement.IntakeTargetState;
import frc.util.StateManagement.LEDSignalTargetState;
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
  public static final LimelightSubsystem LIMELIGHT_SUBSYSTEM_ONE = new LimelightSubsystem("limelight");
  public static final LimelightSubsystem LIMELIGHT_SUBSYSTEM_TWO = new LimelightSubsystem("limelight-two");
  public static final LimelightSubsystem LIMELIGHT_SUBSYSTEM_THREE = new LimelightSubsystem("limelight-three");
  public static final LimelightSubsystem LIMELIGHT_SUBSYSTEM_FOUR = new LimelightSubsystem("limelight-four");
  public static final DrivetrainSubsystem DRIVETRAIN_SUBSYSTEM = new DrivetrainSubsystem();
  public static final PoseEstimatorSubsystem POSE_ESTIMATOR_SUBSYSTEM = new PoseEstimatorSubsystem();
  public static final CommandXboxController DRIVE_CONTROLLER = new CommandXboxController(RobotMap.DRIVE_CONTROLLER_PORT);
  public static DriverStation.Alliance allianceColor = Alliance.Blue;
  public static final DrivetrainDefaultCommand DRIVETRAIN_DEFAULT_COMMAND = new DrivetrainDefaultCommand();
  public static final ReactDashSubsystem REACT_DASH_SUBSYSTEM = new ReactDashSubsystem();
  public static final AutoChooserSubsystemReact AUTO_CHOOSER = new AutoChooserSubsystemReact();
  public static final TeleopDashboardSubsystem TELEOP_DASHBOARD_SUBSYSTEM = new TeleopDashboardSubsystem();
  public static final ButtonCode BUTTON_CODE = new ButtonCode();
  public static final IntakeSubsystem INTAKE_SUBSYSTEM = new IntakeSubsystem();
  public static final ShooterSubsystem SHOOTER_SUBSYSTEM = new ShooterSubsystem();
  public static final ElbowSubsystem ELBOW_SUBSYSTEM = new ElbowSubsystem();
  public static final WristSubsystem WRIST_SUBSYSTEM = new WristSubsystem();
  // States
  public static ScoringTargetState SCORING_TARGET_STATE = ScoringTargetState.SPEAKER;
  public static IntakeTargetState INTAKE_TARGET_STATE = IntakeTargetState.GROUND;
  public static LEDSignalTargetState LED_SIGNAL_TARGET_STATE = LEDSignalTargetState.NONE;
  public static TrapSourceLaneTargetState TRAP_SOURCE_LANE_TARGET_STATE = TrapSourceLaneTargetState.CENTER;
  public static ArmUpTargetState ARM_UP_TARGET_STATE = ArmUpTargetState.FREE;
  public static ShooterRevTargetState SHOOTER_REV_TARGET_STATE = ShooterRevTargetState.OFF;
  public static ClimbPositionTargetState CLIMB_POSITION_TARGET_STATE = ClimbPositionTargetState.LEFT_CLOSE;

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
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    DRIVETRAIN_SUBSYSTEM.setDefaultCommand(DRIVETRAIN_DEFAULT_COMMAND);
    ELBOW_SUBSYSTEM.setDefaultCommand(new ElbowDefaultCommand());

    new Trigger(() -> DRIVE_CONTROLLER.getHID().getLeftBumper()
        && (DRIVE_CONTROLLER.getHID().getRightBumper())
        && (DRIVE_CONTROLLER.getHID().getYButton()))
        .onTrue(new InstantCommand(() -> DRIVETRAIN_SUBSYSTEM.zeroGyroscope()));
    AUTO_CHOOSER.ShowTab();

    // Test button that changes the score target to trap
    /*new Trigger(() -> DRIVE_CONTROLLER.getHID().getAButton()).toggleOnTrue(
      new InstantCommand(() -> {SCORING_TARGET_STATE = ScoringTargetState.TRAP; System.out.println("Clicked A");})
    );*/

    // intake with left trigger
    DRIVE_CONTROLLER.leftBumper().whileTrue(new IntakeCommand(INTAKE_SUBSYSTEM));

    // shoot with right trigger (must have piece)
    // new Trigger(() -> SHOOTER_SUBSYSTEM.hasPiece() && DRIVE_CONTROLLER.getHID().getLeftBumper()).onTrue(new ShootCommand());

    DRIVE_CONTROLLER.rightBumper().whileTrue(new StartEndCommand(
      () -> SHOOTER_SUBSYSTEM.setPowerManually(.5),
      () -> SHOOTER_SUBSYSTEM.setPowerManually(0),
      SHOOTER_SUBSYSTEM));
    /*
    DRIVE_CONTROLLER.y().whileTrue(new StartEndCommand(
      () -> WRIST_SUBSYSTEM.setPowerManually(.2),
      () -> WRIST_SUBSYSTEM.setPowerManually(0), 
      WRIST_SUBSYSTEM));

    DRIVE_CONTROLLER.a().whileTrue(new StartEndCommand(
      () -> WRIST_SUBSYSTEM.setPowerManually(-.2),
      () -> WRIST_SUBSYSTEM.setPowerManually(0),
      WRIST_SUBSYSTEM));
    */
    DRIVE_CONTROLLER.y().whileTrue(new ElbowGoToPositionCommand(-25));
    DRIVE_CONTROLLER.a().whileTrue(new ElbowGoToPositionCommand(-2));
    

    DRIVE_CONTROLLER.x().whileTrue(new StartEndCommand(
      () -> ELBOW_SUBSYSTEM.setPowerManually(.1),
      () -> ELBOW_SUBSYSTEM.setPowerManually(0), 
      ELBOW_SUBSYSTEM));

    DRIVE_CONTROLLER.b().whileTrue(new StartEndCommand(
      () -> ELBOW_SUBSYSTEM.setPowerManually(-.1),
      () -> ELBOW_SUBSYSTEM.setPowerManually(0), 
      ELBOW_SUBSYSTEM));

    configureButtonBindings();
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
    // allianceColor = DriverStation.getAlliance().get();
    // AUTO_CHOOSER.BuildAutoChooser(allianceColor);
  }

  private void configureButtonBindings() {
    BUTTON_CODE.getButton(Buttons.INTAKE_TRAP_SOURCE)
    .onTrue(new InstantCommand(() -> INTAKE_TARGET_STATE = IntakeTargetState.TRAP_SOURCE));
  }
}
