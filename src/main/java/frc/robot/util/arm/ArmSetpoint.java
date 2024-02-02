// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.arm;

import frc.robot.Constants;

/** Add your docs here. */
public class ArmSetpoint {
    private String name;
    private double wristRotationSetpoint;
    private double armRotationSetpoint;

    public ArmSetpoint(String name, double wristRotationSetpoint, double armRotationSetpoint) {
        this.name = name;
        this.armRotationSetpoint = armRotationSetpoint;
        this.wristRotationSetpoint = wristRotationSetpoint;
    }
    public String getName() {
        return name;
    }
    
    public double getWristRotationSetpoint() {
        return wristRotationSetpoint;
    }

    public double getWristRotationSetpointDegrees() {
        return wristRotationSetpoint / Constants.ARM_DEGREES_TO_ENCODER_UNITS;
    }

    public double getArmRotationSetpoint() {
        return armRotationSetpoint;
    }

    public double getArmRotationSetpointDegrees() {
        return armRotationSetpoint / Constants.ARM_DEGREES_TO_ENCODER_UNITS;
    }

    public void setWristRotationSetpoint(double newWristRotationSetpoint) {
        this.wristRotationSetpoint = newWristRotationSetpoint;
    }

    public void setArmRotationSetpoint(double newArmRotationSetpoint) {
        this.armRotationSetpoint = newArmRotationSetpoint;
    }


}
