package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Constants.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.util.Slew;

public class DrivetrainAutoAimNoteCommand extends Command {
    public PIDController _autoAlignRotationPID = new PIDController(0.17,  0, 4);
    public PIDController _yPID = new PIDController(.022, 0, .02);
    private ChassisSpeeds _chassisSpeeds = new ChassisSpeeds(0,0,0);
    private double _velocityXSlewRate = DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / Constants.SLEW_FRAMES_TO_MAX_X_VELOCITY;
    private double _velocityYSlewRate = DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / Constants.SLEW_FRAMES_TO_MAX_Y_VELOCITY;
    private double _angularSlewRate = DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / Constants.SLEW_FRAMES_TO_MAX_ANGULAR_VELOCITY;

    public boolean CutPower = false;

    public DrivetrainAutoAimNoteCommand() {
        addRequirements(Robot.DRIVETRAIN_SUBSYSTEM);
        _autoAlignRotationPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    private double getAngularVelocityForAlignment() {
        double txRadians = Math.toRadians(Robot.LIMELIGHT_PICKUP.getBufferedTx());
        double angularVelocity = _autoAlignRotationPID.calculate(txRadians) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND; 
        return angularVelocity;
    }

    @Override
    public void initialize() {
        _chassisSpeeds = new ChassisSpeeds(0, 0, 0);
        _autoAlignRotationPID.reset();
        _yPID.reset();
    }

    @Override
    public void execute() {
        double r = getAngularVelocityForAlignment();
        double txMaxValue = 27;
        double tx = Robot.LIMELIGHT_PICKUP.getBufferedTx();
        double ta = Robot.LIMELIGHT_PICKUP.getBufferedTa();

        var proportion = (txMaxValue - Math.abs(tx)) / txMaxValue;
        double minSpeed = 0;
        double additionalSpeed = 3;
        
        //var txSign = Math.copySign(1, tx);
        //double y = _yPID.calculate(Math.min(Math.abs(tx), txMaxValue / 2));
        //y = y * txSign;

        SmartDashboard.putNumber("limelight pickup angular velocity", r);
        SmartDashboard.putNumber("limelight pickup ta", ta);

        // double distanceFromTargetTurnReduction =  0.8 / ta;
        // r = r * distanceFromTargetTurnReduction;

        var targetChassisSpeeds = new ChassisSpeeds(
            minSpeed + (proportion * additionalSpeed), // x translation
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
