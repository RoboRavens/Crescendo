// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.awt.Button;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.drivetrain.DrivetrainDefaultCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.TrajectoryGenerationSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  public static final LimelightHelpers LIMELIGHT_HELPERS = new LimelightHelpers();
  public static final LimelightSubsystem LIMELIGHT_SUBSYSTEM_ONE = new LimelightSubsystem("limelight");
  public static final LimelightSubsystem LIMELIGHT_SUBSYSTEM_TWO = new LimelightSubsystem("limelight-two");
  public static final LimelightSubsystem LIMELIGHT_SUBSYSTEM_THREE = new LimelightSubsystem("limelight-three");
  public static final XboxController XBOX_CONTROLLER2 = new XboxController(0); // 0 is the USB Port to be used as indicated on the Driver Station

  public static final TrajectoryGenerationSubsystem TRAJECTORY_GENERATION_SUBSYSTEM = new TrajectoryGenerationSubsystem();
  public static final DrivetrainSubsystem DRIVETRAIN_SUBSYSTEM = new DrivetrainSubsystem();
  public static final PoseEstimatorSubsystem POSE_ESTIMATOR_SUBSYSTEM = new PoseEstimatorSubsystem();
  public static final XboxController XBOX_CONTROLLER = new XboxController(0);
  public static DriverStation.Alliance allianceColor = Alliance.Blue;
  public static final DrivetrainDefaultCommand DRIVETRAIN_DEFAULT_COMMAND = new DrivetrainDefaultCommand();
  private static final double TRAJECTORY_CONFIG_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
  private static final double TRAJECTORY_CONFIG_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND = Math.PI * .75;

  public static final TrapezoidProfile.Constraints SWERVE_CONTROLLER_ANGULAR_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          TRAJECTORY_CONFIG_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, TRAJECTORY_CONFIG_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND);

          
  public Robot() {}

  @Override
  public void robotPeriodic() {
    allianceColor = DriverStation.getAlliance().get();
    CommandScheduler.getInstance().run();
    Robot.TRAJECTORY_GENERATION_SUBSYSTEM.generateTrajectory();
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    DRIVETRAIN_SUBSYSTEM.setDefaultCommand(DRIVETRAIN_DEFAULT_COMMAND);
    Field2d m_field = new Field2d();
    SmartDashboard.putData("Field", m_field);
    m_field.setRobotPose(Robot.POSE_ESTIMATOR_SUBSYSTEM.getCurrentPose());
    m_field.getObject("traj").setTrajectory(Robot.TRAJECTORY_GENERATION_SUBSYSTEM.generateTrajectory());
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
   
    if (XBOX_CONTROLLER.getAButton()) {
       Robot.DRIVETRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(Robot.TRAJECTORY_GENERATION_SUBSYSTEM.generateTrajectory());
    } 
   
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
