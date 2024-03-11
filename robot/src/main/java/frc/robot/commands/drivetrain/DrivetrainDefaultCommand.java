package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.Constants.Constants;
import frc.util.AngularPositionHolder;
import frc.util.Deadband;
import frc.util.Slew;
import frc.util.StateManagement.DrivetrainState;
import frc.util.StateManagement.OverallState;

public class DrivetrainDefaultCommand extends Command {
    private ChassisSpeeds _chassisSpeeds = new ChassisSpeeds(0,0,0);
    private double _velocityXSlewRate = DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / Constants.SLEW_FRAMES_TO_MAX_X_VELOCITY;
    private double _velocityYSlewRate = DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / Constants.SLEW_FRAMES_TO_MAX_Y_VELOCITY;
    private double _angularSlewRate = DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / Constants.SLEW_FRAMES_TO_MAX_ANGULAR_VELOCITY;

    public boolean CutPower = false;

    public DrivetrainDefaultCommand() {
        addRequirements(Robot.DRIVETRAIN_SUBSYSTEM);
    }

    @Override
    public void initialize() {
        // AngularPositionHolder.GetInstance().reset();
        _chassisSpeeds = new ChassisSpeeds(0, 0, 0);
    }

    @Override
    public void execute() {
        double controllerDirection = Robot.allianceColor == Alliance.Red ? 1 : -1;
        double x = Robot.DRIVE_CONTROLLER.getLeftY() * controllerDirection;
        double y = Robot.DRIVE_CONTROLLER.getLeftX() * controllerDirection;
        double r = Robot.DRIVE_CONTROLLER.getRightX() * -1;

        if (Robot.cutPower) {
            x *= 0.5;
            y *= 0.5;
            r *= 0.5;
        }
        
        Rotation2d a = Robot.DRIVETRAIN_SUBSYSTEM.getOdometryRotation(); // The angle of the robot as measured by a gyroscope. The robot's angle is considered to be zero when it is facing directly away from your alliance station wall.        
        
        if (Robot.DRIVETRAIN_STATE == DrivetrainState.FREEHAND) {
            x = Deadband.adjustValueToZero(x, Constants.JOYSTICK_DEADBAND);
            y = Deadband.adjustValueToZero(y, Constants.JOYSTICK_DEADBAND);
            r = Deadband.adjustValueToZero(r, Constants.JOYSTICK_DEADBAND);

            x = x * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
            y = y * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
            r = r * Constants.DRIVE_MAX_TURN_RADIANS_PER_SECOND;

            r = AngularPositionHolder.GetInstance().getAngularVelocity(r, a.getRadians());

            var targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                x, // x translation
                y, // y translation
                r, // rotation
                a // The angle of the robot as measured by a gyroscope.
            );

            targetChassisSpeeds.vxMetersPerSecond = Slew.GetSlewedTarget(_velocityXSlewRate, targetChassisSpeeds.vxMetersPerSecond, _chassisSpeeds.vxMetersPerSecond);
            targetChassisSpeeds.vyMetersPerSecond = Slew.GetSlewedTarget(_velocityYSlewRate, targetChassisSpeeds.vyMetersPerSecond, _chassisSpeeds.vyMetersPerSecond);
            targetChassisSpeeds.omegaRadiansPerSecond = Slew.GetSlewedTarget(_angularSlewRate, targetChassisSpeeds.omegaRadiansPerSecond, _chassisSpeeds.omegaRadiansPerSecond);
            _chassisSpeeds = targetChassisSpeeds;

            Robot.DRIVETRAIN_SUBSYSTEM.drive(targetChassisSpeeds);
        }
        else if (Robot.DRIVETRAIN_STATE == DrivetrainState.ROBOT_ALIGN) {
            if (Robot.OVERALL_STATE == OverallState.LOADED_TRANSIT) {
               
            }
        }
        
    }

    // public double getYVelocity(Translation2d target) {
    //     if (target == null) {
    //         SmartDashboard.putNumber("Score Target Y", -1);
    //         return 0;
    //     }

        
    //     SmartDashboard.putNumber("Score Target Y", target.getY());
    //     double yOffsetFromTarget = target.getY() - Robot.POSE_ESTIMATOR_SUBSYSTEM.getCurrentPose().getY();
        
    //     if (Math.abs(yOffsetFromTarget) < Constants.ROBOT_IS_ALIGNED_ERROR_MARGIN_METERS) {
    //         return 0;
    //     }

    //     double ySpeed = _yPID.calculate(yOffsetFromTarget) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * -1;
    //     double velocityDirection = ySpeed < 0 ? -1 : 1;
    //     if (Math.abs(ySpeed) > DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / 2) {
    //         ySpeed = DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / 2 * velocityDirection;
    //     } 
    //     // If the y velocity is less than 0.2 and the robot is not yet within 0.5 inches from the target y location (exact value should be updated)
    //     else if (Math.abs(ySpeed) < 0.2 && Math.abs(yOffsetFromTarget) > 0.0127) {
    //         ySpeed = 0.2 * velocityDirection;
    //     }
    //     // If the offset is within 0.5 inches, set the speed to 0 (exact value should be updated)
    //     else if (Math.abs(yOffsetFromTarget) < 0.0127) {
    //         ySpeed = 0;
    //     }
    //     return ySpeed;
    // }

    // public double getXVelocity(Translation2d target) {
    //     if (target == null) {
    //         SmartDashboard.putNumber("Score Target X", -1);
    //         return 0;
    //     }

    //     SmartDashboard.putNumber("Score Target X", target.getX());
    //     double xOffsetFromTarget = target.getX() - Robot.POSE_ESTIMATOR_SUBSYSTEM.getCurrentPose().getX();
        
    //     if (Math.abs(xOffsetFromTarget) < Constants.ROBOT_IS_ALIGNED_ERROR_MARGIN_METERS) {
    //         return 0;
    //     }
        
    //     double xSpeed = _xPID.calculate(xOffsetFromTarget) * Robot.DRIVE_TRAIN_SUBSYSTEM.MAX_VELOCITY_METERS_PER_SECOND * -1;
    //     double velocityDirection = xSpeed < 0 ? -1 : 1;
    //     if (Math.abs(xSpeed) > DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / 2) {
    //         xSpeed = DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / 2 * velocityDirection;
    //     }
    //     else if (Math.abs(xSpeed) < 0.2 && Math.abs(xOffsetFromTarget) > 0.0127) {
    //         xSpeed = 0.2 * velocityDirection;
    //     }
    //     // If the offset is within 0.5 inches, set the speed to 0 (exact value should be updated)
    //     else if (Math.abs(xOffsetFromTarget) < 0.0127) {
    //         xSpeed = 0;
    //     }
    //     return xSpeed;
    // }

    // private double getAngularVelocityForAlignment(double targetRotation) {
    //     // Assumes that the robot's initial rotation (0) is aligned with the scoring nodes
    //     double currentRotation = Robot.DRIVE_TRAIN_SUBSYSTEM.getOdometryRotation().getRadians();
    //     double rotationOffset = currentRotation - targetRotation;
    //     //SmartDashboard.putNumber("Rotation Offset", rotationOffset);
    //     double angularVelocity = _scoringRotationAlignPID
    //     .calculate(rotationOffset) 
    //     * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    //     double velocityDirection = angularVelocity < 0 ? -1 : 1;
    //     boolean isWithinTwoHundredthsRadianOfTargetRotation = currentRotation > targetRotation - 0.02 && currentRotation < targetRotation + 0.02;
    //     //SmartDashboard.putBoolean("isWithinTwoHundredthsRadianOfTargetRotation", isWithinTwoHundredthsRadianOfTargetRotation);
    //     // If the angular velocity is greater than the max angular velocity, set it to the max angular velocity
    //     if (Math.abs(angularVelocity) > DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND) {
    //         return DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * velocityDirection;
    //     }
    //     // If the angular velocity is less than 0.4 and the robot is not within 0.02 radians of 0 degrees, set the velocity to 0.4
    //     else if (Math.abs(angularVelocity) < 0.4 && isWithinTwoHundredthsRadianOfTargetRotation == false) {
    //         return 0.4 * velocityDirection;
    //     }
    //     return angularVelocity; // angular velocity
    // }

    @Override
    public void end(boolean interrupted) {
        Robot.DRIVETRAIN_SUBSYSTEM.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        //SmartDashboard.putString("end method", "end");
    }

    private double getDegreesToMovementDirection(double x, double y, double robotAngleDegrees) {
        double desiredAngleRadians = Math.atan2(y, x);
        return this.getShortestAngularDifference(robotAngleDegrees, Math.toDegrees(desiredAngleRadians));
    }

    /**
     * Gets the shortest angular difference between two points.
     * @param current current angle in degrees
     * @param target target angle in degrees
     * @return the shortest angle to get from current to target
     */
    private double getShortestAngularDifference(double current, double target) {
		current = current % 360.0;
		target = target % 360.0;
		double d = Math.abs(current - target) % 360.0; 
		double r = d > 180 ? 360 - d : d;
		
		//calculate sign 
		int sign = (current - target >= 0 && current - target <= 180) || (current - target <= -180 && current - target >= -360) ? 1 : -1; 
		r *= sign;

		return r;
    }
}