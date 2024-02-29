// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.arm;

/** Add your docs here. */
public class LimbSetpoint {
    public static LimbSetpoint GROUND_PICKUP = new LimbSetpoint("", 0, 0);
    public static LimbSetpoint AMP_SCORING = new LimbSetpoint("", 0, 0);
    public static LimbSetpoint TRAP_SCORING = new LimbSetpoint("", 0, 0);
    public static LimbSetpoint AMP_AND_SPEAKER_SOURCE_INTAKE = new LimbSetpoint("", 0, 0);
    public static LimbSetpoint TRAP_SOURCE_INTAKE = new LimbSetpoint("", 0, 0);

    private String _name;
    private int _elbowRotationPosition;
    private int _wristRotationPosition;

    private LimbSetpoint(String name, int elbowRotationSetpoint, int wristRotationPosition) {
        _name = name;
        _elbowRotationPosition = elbowRotationSetpoint;
        _wristRotationPosition = wristRotationPosition;
    }

    public String getName() {
        return _name;
    }
    
    public double getElbowRotationPosition() {
        return _elbowRotationPosition;
    }

    public double getWristRotationPosition() {
        return _wristRotationPosition;
    }
}
