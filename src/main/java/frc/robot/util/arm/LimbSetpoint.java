// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.arm;

/** Add your docs here. */
public class LimbSetpoint {
    public static LimbSetpoint GROUND_PICKUP = new LimbSetpoint("", 0, 0);

    private String _name;
    private int _armRotationPosition;
    private int _wristRotationPosition;

    private LimbSetpoint(String name, int armRotationSetpoint, int wristRotationPosition) {
        _name = name;
        _armRotationPosition = armRotationSetpoint;
        _wristRotationPosition = wristRotationPosition;
    }

    public String getName() {
        return _name;
    }
    
    public double getArmRotationPosition() {
        return _armRotationPosition;
    }

    public double getWristRotationPosition() {
        return _wristRotationPosition;
    }
}
