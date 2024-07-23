package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.Constants.Constants;
import frc.util.AngularPositionHolder;
import frc.util.Deadband;
import frc.util.Slew;
import frc.util.StateManagement.DrivetrainState;

public class DrivetrainDefaultCommand extends Command {
    private ChassisSpeeds _chassisSpeeds = new ChassisSpeeds(0,0,0);
    private double _velocityXSlewRate = DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / Constants.SLEW_FRAMES_TO_MAX_X_VELOCITY;
    private double _velocityYSlewRate = DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / Constants.SLEW_FRAMES_TO_MAX_Y_VELOCITY;
    private double _angularSlewRate = DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / Constants.SLEW_FRAMES_TO_MAX_ANGULAR_VELOCITY;
    public PIDController _scoringRotationAlignPID = new PIDController(7, 0, 4);

    private Timer _xingTimer = new Timer();

    private Timer _bufferedTargetAngleTimer = new Timer();
    private double _bufferedTargetAngle = 0;
    private boolean _visionAligned = false;

    public boolean CutPower = false;

    public DrivetrainDefaultCommand() {
        addRequirements(Robot.DRIVETRAIN_SUBSYSTEM);
        _bufferedTargetAngleTimer.start();
    }

    @Override
    public void initialize() {
        _xingTimer.reset();
        _xingTimer.start();

        AngularPositionHolder.GetInstance().reset();
        _chassisSpeeds = new ChassisSpeeds(0, 0, 0);
    }

    @Override
    public void execute() {
        double controllerDirection = Robot.allianceColor == Alliance.Red ? 1 : -1;
        double x = Robot.DRIVE_CONTROLLER.getLeftY() * controllerDirection;
        double y = Robot.DRIVE_CONTROLLER.getLeftX() * controllerDirection;
        double r = Robot.DRIVE_CONTROLLER.getRightX() * -1;

        x = Deadband.adjustValueToZero(x, Constants.JOYSTICK_DEADBAND);
        y = Deadband.adjustValueToZero(y, Constants.JOYSTICK_DEADBAND);
        r = Deadband.adjustValueToZero(r, Constants.JOYSTICK_DEADBAND);
        
        // The angle of the robot as measured by a gyroscope. The robot's angle is considered to be zero when it is facing directly away from your alliance station wall.        
        Rotation2d robotRotation = Robot.DRIVETRAIN_SUBSYSTEM.getOdometryRotation();
        
        // If no joystick input OR robot is being auto-driven, reset the x-ing timer.
        if ((x == 0 && y == 0 && r == 0) == false || (Robot.DRIVETRAIN_STATE != DrivetrainState.FREEHAND)) {
            _xingTimer.reset();
        }

        var txDegreesSD = Robot.LIMELIGHT_BACK.getTx();
        SmartDashboard.putNumber("txDegrees", txDegreesSD);
        SmartDashboard.putNumber("tyDegrees", Robot.LIMELIGHT_BACK.getTy());
        var targetAngleDegreesSD = robotRotation.getDegrees() - txDegreesSD;
        SmartDashboard.putNumber("targetAngleDegrees", targetAngleDegreesSD);
        SmartDashboard.putNumber("_bufferedTargetAngle", _bufferedTargetAngle);
        
        SmartDashboard.putBoolean("LL BACK Has Target", Robot.LIMELIGHT_BACK.hasVisionTargetBoolean());
        SmartDashboard.putNumber("_bufferedTargetAngleTimer", _bufferedTargetAngleTimer.get());
        boolean visionAligned = false;
        
        if (_xingTimer.get() >= Constants.DRIVETRAIN_HOLD_POSITION_TIMER_THRESHOLD) {
            Robot.DRIVETRAIN_SUBSYSTEM.holdPosition();
        } else {
            double cutPowerRotation = Robot.cutPower ? 0.5 : 1;
            double cutPowerTranslation = Robot.cutPower ? 0.25 : 1;
            x = x * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * cutPowerTranslation;
            y = y * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * cutPowerTranslation;
            r = r * Constants.DRIVE_MAX_TURN_RADIANS_PER_SECOND * cutPowerRotation;

            if (Robot.DRIVETRAIN_STATE == DrivetrainState.ROBOT_ALIGN) {
              // From the docs it sounds like this boolean will ONLY be true if it sees
              // the priority tag, but we will have to double check to be certain.
              if (Robot.LIMELIGHT_BACK.hasVisionTargetBoolean()) {
                var txDegrees = Robot.LIMELIGHT_BACK.getTx();
                var targetAngleDegrees = robotRotation.getDegrees() - txDegrees;
                _bufferedTargetAngle = Math.toRadians(targetAngleDegrees);
                _bufferedTargetAngleTimer.reset();
              }

              // track to the last known target rotation for .5 seconds after losing vision of the target
              if (_bufferedTargetAngleTimer.get() < .5) {
                var diff = Math.toDegrees(_bufferedTargetAngle) - robotRotation.getDegrees();
                visionAligned = Math.abs(diff) < Constants.VISION_ALIGNED_TX_BUFFER_DEGREES;
                r = getAngularVelocityForAlignment(_bufferedTargetAngle);
              }
            }

            // angular position holder only acts if r == 0
            r = AngularPositionHolder.GetInstance().getAngularVelocity(r, robotRotation.getRadians());

            var targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                x, // x translation
                y, // y translation
                r, // rotation
                robotRotation // The angle of the robot as measured by a gyroscope.
            );

            targetChassisSpeeds.vxMetersPerSecond = Slew.GetSlewedTarget(_velocityXSlewRate, targetChassisSpeeds.vxMetersPerSecond, _chassisSpeeds.vxMetersPerSecond);
            targetChassisSpeeds.vyMetersPerSecond = Slew.GetSlewedTarget(_velocityYSlewRate, targetChassisSpeeds.vyMetersPerSecond, _chassisSpeeds.vyMetersPerSecond);
            targetChassisSpeeds.omegaRadiansPerSecond = Slew.GetSlewedTarget(_angularSlewRate, targetChassisSpeeds.omegaRadiansPerSecond, _chassisSpeeds.omegaRadiansPerSecond);
            _chassisSpeeds = targetChassisSpeeds;

            Robot.DRIVETRAIN_SUBSYSTEM.drive(targetChassisSpeeds);
        }
        
        _visionAligned = visionAligned;
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

    private double getAngularVelocityForAlignment(double targetRotation) {
        // Assumes that the robot's initial rotation (0) is aligned with the scoring nodes
        SmartDashboard.putNumber("APH Target Rotation", targetRotation);
        double currentRotation = Robot.DRIVETRAIN_SUBSYSTEM.getOdometryRotation().getRadians();
        double rotationOffset = currentRotation - targetRotation;
        SmartDashboard.putNumber("APH Rotation Offset", rotationOffset);
        SmartDashboard.putNumber("APH Rotation Offset Degrees", Math.toDegrees(rotationOffset));
        double angularVelocity = _scoringRotationAlignPID.calculate(rotationOffset);
        if (Math.abs(rotationOffset) > Constants.VISION_ALIGNED_TX_BUFFER_DEGREES) {
          double sign = Math.copySign(1, angularVelocity);
          angularVelocity += Constants.VALUE_TO_BREAK_ALIGNMENT_STATIC_FRICTION * sign;
        } else {
          angularVelocity = 0;
        }
        return angularVelocity;
    }

    // private double getAngularVelocityForAlignmentFromRadiansOffset(double radiansOffset) {
    //     double targetRotation = Robot.DRIVETRAIN_SUBSYSTEM.getOdometryRotation().getRadians() + radiansOffset;

    //     return getAngularVelocityForAlignment(targetRotation);
    // }

    @Override
    public void end(boolean interrupted) {
        Robot.DRIVETRAIN_SUBSYSTEM.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        _visionAligned = false;
        //SmartDashboard.putString("end method", "end");
    }

    public boolean isVisionAligned() {
      return _visionAligned;
    }

    // private double getDegreesToMovementDirection(double x, double y, double robotAngleDegrees) {
    //     double desiredAngleRadians = Math.atan2(y, x);
    //     return this.getShortestAngularDifference(robotAngleDegrees, Math.toDegrees(desiredAngleRadians));
    // }

    /**
     * Gets the shortest angular difference between two points.
     * @param current current angle in degrees
     * @param target target angle in degrees
     * @return the shortest angle to get from current to target
     */
    // private double getShortestAngularDifference(double current, double target) {
		// current = current % 360.0;
		// target = target % 360.0;
		// double d = Math.abs(current - target) % 360.0; 
		// double r = d > 180 ? 360 - d : d;
		
		// //calculate sign 
		// int sign = (current - target >= 0 && current - target <= 180) || (current - target <= -180 && current - target >= -360) ? 1 : -1; 
		// r *= sign;

		// return r;
    // }
}