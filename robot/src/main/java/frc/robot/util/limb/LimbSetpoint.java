// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.limb;

import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.util.Constants.ElbowConstants;
import frc.robot.util.Constants.WristConstants;

public class LimbSetpoint {
    public static LimbSetpoint GROUND_PICKUP = new LimbSetpoint("Ground Pickup", ElbowConstants.DEGREES_GROUND_PICKUP,WristConstants.DEGREES_GROUND_PICKUP);
    public static LimbSetpoint SPEAKER_SCORING = new LimbSetpoint("Speaker Scoring", ElbowConstants.DEGREES_SPEAKER_SCORING, WristConstants.DEGREES_SPEAKER_SCORING);
    public static LimbSetpoint SPEAKER_SCORING_ARM_UP = new LimbSetpoint("Speaker Scoring Arm Up", ElbowConstants.DEGREES_SPEAKER_SCORING_ARM_UP, WristConstants.DEGREES_SPEAKER_SCORING_ARM_UP);
    public static LimbSetpoint DEFENDED_SPEAKER_SCORING = new LimbSetpoint("Defended Speaker Scoring", 0, 0);
    public static LimbSetpoint AMP_SCORING = new LimbSetpoint("Amp Scoring", ElbowConstants.DEGREES_AMP_SCORE, WristConstants.DEGREES_AMP_SCORE);
    public static LimbSetpoint TRAP_SCORING = new LimbSetpoint("Trap Scoring", ElbowConstants.DEGREES_TRAP_SCORE , WristConstants.DEGREES_TRAP_SCORE);
    public static LimbSetpoint AMP_AND_SPEAKER_SOURCE_INTAKE = new LimbSetpoint("DualSource Intake", 0, 0);
    public static LimbSetpoint TRAP_SOURCE_INTAKE = new LimbSetpoint("TrapSource Intake", ElbowConstants.DEGREES_TRAP_LOAD_FROM_SOURCE, WristConstants.DEGREES_TRAP_LOAD_FROM_SOURCE);


    private String _name;
    private double _elbowRotationPosition;
    private double _wristRotationPosition;

    public LimbSetpoint(String name, double elbowRotationSetpoint, double wristRotationPosition) {
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
