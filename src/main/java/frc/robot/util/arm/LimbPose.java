// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.arm;

import frc.robot.Constants;

/** Add your docs here. */
public class LimbPose {
    private double elbowAngleDegrees;
    private double elbowAngleRadians;
    private double wristAngleDegrees;
    private double wristAngleRadians;
    private double armLengthToHitConstraintNativeUnits;
    private double elbowRotationMinimumBoundNativeUnits;
    private double elbowRotationMaximumBoundNativeUnits;
    private double wristRotationMinimumBoundNativeUnits;
    private double wristRotationMaximumBoundNativeUnits;

    //double for additional length on arm
    private double armRotationMinimumBoundDegrees;
    private double armRotationMaximumBoundDegrees;
    private double armNetLengthInches;
    private double armNetHeightInches;
    private double wristNetExtensionInches;
    private double armNetLengthNativeUnits;
    private double armNetHeightNativeUnits;
    private double armNetExtensionNativeUnits;
    private double armLengthToHitConstraintInches;
    private double heightAtMaxExtensionInches;
    private double widthAtMaxExtensionInches;


    public LimbPose(double startingDegress) {
        this.setElbowAngleDegrees(startingDegress);
        this.setWristAngleDegrees(startingDegress);
    }


    //calculating arm position making sure to keep it within the robots legal bounds (code from extension arm in Charged up, change to account for wrist)
    public void calculateInstantaneousMaximums() {
        calculateInstantaneousMaxElbowRotation();
        calculateInstantaneousMaxWristRotation();
    }

    public void calculateInstantaneousMaxElbowRotation() {
       // AngularConstraintWindow window = getMaxRotationAngularConstraintWindow(wristNetExtensionInches, armAngleDegrees);

       // this.armRotationMinimumBoundDegrees = window.getLowerBound();
       // this.armRotationMinimumBoundNativeUnits = window.getLowerBound() * Constants.ARM_DEGREES_TO_ENCODER_UNITS;
       // this.armRotationMaximumBoundDegrees = window.getUpperBound();
       // this.armRotationMaximumBoundNativeUnits = window.getUpperBound() * Constants.ARM_DEGREES_TO_ENCODER_UNITS;

    }


    //alter to account for not extending but rotating or wrist
    public void calculateInstantaneousMaxWristRotation() {
       // ArmMaximumConstraint maxConstraintAtAngle = calculateMaxExtensionAtAngleDegrees(armAngleDegrees);

       // this.armLengthToHitConstraintInches = maxConstraintAtAngle.getArmLengthToHitConstraintInches();
       // this.armLengthToHitConstraintNativeUnits = maxConstraintAtAngle.getArmLengthToHitConstraintNativeUnits();
       // this.heightAtMaxExtensionInches = maxConstraintAtAngle.getHeightAtMaxExtensionInches();
       // this.widthAtMaxExtensionInches = maxConstraintAtAngle.getWidthAtMaxExtensionInches();
    }
    //^actual code for extension version in 2023, incomplete math for full limb movement

    public double getElbowAngleRadians() {
        return elbowAngleRadians;
    }

    public double getElbowRotationNativeUnits() {
        return elbowAngleDegrees * Constants.ELBOW_DEGREES_TO_ENCODER_UNITS;
    }
    public double getWristRotationNativeUnits() {
        return wristAngleDegrees * Constants.WRIST_DEGREES_TO_ENCODER_UNITS;

    }
    public void setElbowAngleRadians(double armAngleRadians) {
        this.elbowAngleRadians = armAngleRadians;
        this.elbowAngleDegrees = Math.toDegrees(armAngleRadians);
    }

    public void setElbowAngleDegrees(double armAngleDegrees) {
        this.elbowAngleDegrees = armAngleDegrees;
        this.elbowAngleRadians = Math.toRadians(armAngleDegrees);
    }

    public void setWristAngleDegrees(double wristAngleDegrees) {
        this.wristAngleDegrees = wristAngleDegrees;
        this.wristAngleRadians = Math.toRadians(wristAngleDegrees);
    }

    public double getElbowRotationMinimumBoundNativeUnits() {
        return elbowRotationMinimumBoundNativeUnits;
    }

    public double getElbowRotationMaximumBoundNativeUnits() {
        return elbowRotationMaximumBoundNativeUnits;
    }

    public double getWristRotationMaximumBoundNativeUnits() {
        return wristRotationMinimumBoundNativeUnits;
    }

    public double getWristRotationMinimumBoundNativeUnits() {
        return wristRotationMaximumBoundNativeUnits;
    }

}
