package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Constants.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.util.Deadband;
import frc.util.Slew;

public class DrivetrainAutoAimCommand extends Command {
    public PIDController _autoAlignRotationPID = new PIDController(0.35,  0, 0);
    private ChassisSpeeds _chassisSpeeds = new ChassisSpeeds(0,0,0);
    private double _velocityXSlewRate = DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / Constants.SLEW_FRAMES_TO_MAX_X_VELOCITY;
    private double _velocityYSlewRate = DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / Constants.SLEW_FRAMES_TO_MAX_Y_VELOCITY;
    private double _angularSlewRate = DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / Constants.SLEW_FRAMES_TO_MAX_ANGULAR_VELOCITY;

    public boolean CutPower = false;

    public DrivetrainAutoAimCommand() {
        addRequirements(Robot.DRIVETRAIN_SUBSYSTEM);
        _autoAlignRotationPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    private double getAngularVelocityForAlignment() {
        double txRadians = Math.toRadians(LimelightHelpers.getTX("limelight-pickup"));
        double angularVelocity = _autoAlignRotationPID.calculate(txRadians) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        
        double velocityDirection = angularVelocity < 0 ? -1 : 1;
        boolean isWithinTwoHundredthsRadianOfTargetRotation = txRadians > 0.2;
       
        if (Math.abs(angularVelocity) > DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND) {
            return DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * velocityDirection;
        }
        // If the angular velocity is less than 0.4 and the robot is not within 0.02 radians of 0 degrees, set the velocity to 0.4
        else if (Math.abs(angularVelocity) < 0.4 && isWithinTwoHundredthsRadianOfTargetRotation == false) {
            return 0.4 * velocityDirection;
        }
        return angularVelocity; // angular velocity
    }

    @Override
    public void initialize() {
        // AngularPositionHolder.GetInstance().reset();
        _chassisSpeeds = new ChassisSpeeds(0, 0, 0);
    }

    @Override
    public void execute() {
        double controllerDirection = Robot.allianceColor == Alliance.Red ? 1 : -1;
        double x = 0;
        double y = 0;
        double r = getAngularVelocityForAlignment();
       
       
        Rotation2d a = Robot.DRIVETRAIN_SUBSYSTEM.getOdometryRotation(); // The angle of the robot as measured by a gyroscope. The robot's angle is considered to be zero when it is facing directly away from your alliance station wall.
      /* 
        x = Deadband.adjustValueToZero(x, Constants.JOYSTICK_DEADBAND);
        y = Deadband.adjustValueToZero(y, Constants.JOYSTICK_DEADBAND);
        r = Deadband.adjustValueToZero(r, Constants.JOYSTICK_DEADBAND);

        x = x * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
        y = y * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
        r = r * Constants.DRIVE_MAX_TURN_RADIANS_PER_SECOND;
        */
        var targetChassisSpeeds = new ChassisSpeeds(
            Robot.DRIVE_CONTROLLER.getLeftTriggerAxis() * 3, // x translation
             0, // y translation
            r // rotation
           // a // The angle of the robot as measured by a gyroscope.
        );

        targetChassisSpeeds.vxMetersPerSecond = Slew.GetSlewedTarget(_velocityXSlewRate, targetChassisSpeeds.vxMetersPerSecond, _chassisSpeeds.vxMetersPerSecond);
        targetChassisSpeeds.vyMetersPerSecond = Slew.GetSlewedTarget(_velocityYSlewRate, targetChassisSpeeds.vyMetersPerSecond, _chassisSpeeds.vyMetersPerSecond);
        targetChassisSpeeds.omegaRadiansPerSecond = Slew.GetSlewedTarget(_angularSlewRate, targetChassisSpeeds.omegaRadiansPerSecond, _chassisSpeeds.omegaRadiansPerSecond);
        _chassisSpeeds = targetChassisSpeeds;

        Robot.DRIVETRAIN_SUBSYSTEM.drive(targetChassisSpeeds);
    }

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
