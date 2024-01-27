// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.arm;

import frc.robot.Constants;

/** Add your docs here. */
public class ArmPose {
    private double armAngleDegrees;
    private double armAngleRadians;
    private double wristAngleDegrees;
    private double wristAngleRadians;
    private double armLengthToHitConstraintNativeUnits;
    private double armRotationMinimumBoundNativeUnits;
    private double armRotationMaximumBoundNativeUnits;
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


    public ArmPose(double startingDegress) {
        this.setArmAngleDegrees(startingDegress);
        this.setWristAngleDegrees(startingDegress);
    }


    //calculating arm position making sure to keep it within the robots legal bounds (code from extension arm in Charged up, change to account for wrist)
    public void calculateInstantaneousMaximums() {
        calculateInstantaneousMaxArmRotation();
        calculateInstantaneousMaxWristRotation();
    }

    public void calculateInstantaneousMaxArmRotation() {
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

    public double getArmAngleRadians() {
        return armAngleRadians;
    }

    public double getArmRotationNativeUnits() {
        return armAngleDegrees * Constants.ARM_DEGREES_TO_ENCODER_UNITS;

    }

    public void setArmAngleRadians(double armAngleRadians) {
        this.armAngleRadians = armAngleRadians;
        this.armAngleDegrees = Math.toDegrees(armAngleRadians);
    }

    public void setArmAngleDegrees(double armAngleDegrees) {
        this.armAngleDegrees = armAngleDegrees;
        this.armAngleRadians = Math.toRadians(armAngleDegrees);
    }

    public void setWristAngleDegrees(double wristAngleDegrees) {
        this.wristAngleDegrees = wristAngleDegrees;
        this.wristAngleRadians = Math.toRadians(wristAngleDegrees);
    }

    public double getArmRotationMinimumBoundNativeUnits() {
        return armRotationMinimumBoundNativeUnits;
    }

    public double getArmRotationMaximumBoundNativeUnits() {
        return armRotationMaximumBoundNativeUnits;
    }

    public double getWristRotationMaximumBoundNativeUnits() {
        return wristRotationMinimumBoundNativeUnits;
    }

    public double getWristRotationMinimumBoundNativeUnits() {
        return wristRotationMaximumBoundNativeUnits;
    }

}
