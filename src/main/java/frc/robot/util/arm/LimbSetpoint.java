// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.arm;

import frc.robot.Constants;

/** Add your docs here. */
public class LimbSetpoint {
    private String name;
    private double rotationSetpoint;

    public LimbSetpoint(String name, double rotationSetpoint) {
        this.name = name;
        this.rotationSetpoint = rotationSetpoint;
    }
    public String getName() {
        return name;
    }
    
    public double getRotationSetpoint() {
        return rotationSetpoint;
    }

    public double getWristRotationSetpointDegrees() {
        return rotationSetpoint / Constants.ELBOW_DEGREES_TO_ENCODER_UNITS;
    }

    public void setRotationSetpoint(double rotationSetpoint) {
        this.rotationSetpoint = rotationSetpoint;
    }


}
