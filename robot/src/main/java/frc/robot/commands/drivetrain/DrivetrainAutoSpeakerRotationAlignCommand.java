// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.Constants.Constants;

public class DrivetrainAutoSpeakerRotationAlignCommand extends Command {
  public PIDController _scoringRotationAlignPID = new PIDController(0.17,  0, 4);
  public PIDController _yPID = new PIDController(.022, 0, .02);
  private ChassisSpeeds _chassisSpeeds = new ChassisSpeeds(0,0,0);
  private double _angularSlewRate = DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / Constants.SLEW_FRAMES_TO_MAX_ANGULAR_VELOCITY;

  /** Creates a new DrivetrainAutoSpeakerRotationAlignCommand. */
  public DrivetrainAutoSpeakerRotationAlignCommand() {
    addRequirements(Robot.DRIVETRAIN_SUBSYSTEM);
    _scoringRotationAlignPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentRotation = Robot.DRIVETRAIN_SUBSYSTEM.getPoseRotation().getRadians();
    double rotationOffsetFromCenterOfSpeaker = Robot.DRIVETRAIN_SUBSYSTEM.getAngleOffsetFromCenterOfSpeaker();
    double targetRotation = currentRotation - rotationOffsetFromCenterOfSpeaker;
    double r = getAngularVelocityForAlignment(targetRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double getAngularVelocityForAlignment(double targetRotation) {
    // Assumes that the robot's initial rotation (0) is aligned with the scoring nodes
    double currentRotation = Robot.DRIVETRAIN_SUBSYSTEM.getOdometryRotation().getRadians();
    double rotationOffset = currentRotation - targetRotation;
    //SmartDashboard.putNumber("Rotation Offset", rotationOffset);
    double angularVelocity = _scoringRotationAlignPID
    .calculate(rotationOffset) 
    * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    double velocityDirection = angularVelocity < 0 ? -1 : 1;
    boolean isWithinTwoHundredthsRadianOfTargetRotation = currentRotation > targetRotation - 0.02 && currentRotation < targetRotation + 0.02;
    //SmartDashboard.putBoolean("isWithinTwoHundredthsRadianOfTargetRotation", isWithinTwoHundredthsRadianOfTargetRotation);
    // If the angular velocity is greater than the max angular velocity, set it to the max angular velocity
    if (Math.abs(angularVelocity) > DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND) {
        return DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * velocityDirection;
    }
    // If the angular velocity is less than 0.4 and the robot is not within 0.02 radians of 0 degrees, set the velocity to 0.4
    else if (Math.abs(angularVelocity) < 0.4 && isWithinTwoHundredthsRadianOfTargetRotation == false) {
        return 0.4 * velocityDirection;
    }
    return angularVelocity; // angular velocity
  }
}
