package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;

public class TrajectoryGenerationSubsystem {
    
  public Trajectory generateTrajectory() {

    
    var StartingLocation = new Pose2d(Robot.POSE_ESTIMATOR_SUBSYSTEM.getCurrentPose().getX(),Robot.POSE_ESTIMATOR_SUBSYSTEM.getCurrentPose().getY(), 
    (Robot.DRIVETRAIN_SUBSYSTEM.getPoseRotation()));
    var crossScale = new Pose2d(Units.feetToMeters(23.7), Units.feetToMeters(6.8),
        Rotation2d.fromDegrees(-160));

    
     var interiorWaypoints = new ArrayList<Translation2d>();
    
     TrajectoryConfig config = new TrajectoryConfig(Constants.TRAJECTORY_CONFIG_MAX_VELOCITY_METERS_PER_SECOND,Constants.TRAJECTORY_CONFIG_MAX_ACCELERATION_METERS_PER_SECOND);
     config.setReversed(true);

     var trajectory = TrajectoryGenerator.generateTrajectory(
        StartingLocation,
        interiorWaypoints,
        crossScale,
        config);
        return trajectory;

   
  }

 
  
}

