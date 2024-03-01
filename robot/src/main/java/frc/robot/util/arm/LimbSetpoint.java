// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.arm;

public class LimbSetpoint {
    public static LimbSetpoint GROUND_PICKUP = new LimbSetpoint("Ground Pickup", 0, 0);
    public static LimbSetpoint SPEAKER_SCORING = new LimbSetpoint("Speaker Scoring", 0, 0);
    public static LimbSetpoint SPEAKER_SCORING_ARM_UP = new LimbSetpoint("Speaker Scoring", 0, 0);
    public static LimbSetpoint DEFENDED_SPEAKER_SCORING = new LimbSetpoint("Defended Speaker Scoring", 0, 0);
    public static LimbSetpoint AMP_SCORING = new LimbSetpoint("Amp Scoring", 0, 0);
    public static LimbSetpoint TRAP_SCORING = new LimbSetpoint("Trap Scoring", 0, 0);
    public static LimbSetpoint AMP_AND_SPEAKER_SOURCE_INTAKE = new LimbSetpoint("DualSource Intake", 0, 0);
    public static LimbSetpoint TRAP_SOURCE_INTAKE = new LimbSetpoint("TrapSource Intake", 0, 0);


    private String _name;
    private int _elbowRotationPosition;
    private int _wristRotationPosition;

    public LimbSetpoint(String name, int elbowRotationSetpoint, int wristRotationPosition) {
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
