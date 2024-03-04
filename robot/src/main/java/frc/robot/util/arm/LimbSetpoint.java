// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.arm;

import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.util.Constants.ElbowConstants;
import frc.robot.util.Constants.WristConstants;

public class LimbSetpoint {
    public static LimbSetpoint GROUND_PICKUP = new LimbSetpoint("Ground Pickup", ElbowConstants.DEGREES_FLOOR_PICKUP, WristConstants.DEGREES_FLOOR_PICKUP);
    public static LimbSetpoint SPEAKER_SCORING = new LimbSetpoint("Speaker Scoring", ElbowConstants.DEGREES_FLOOR_PICKUP, WristConstants.DEGREES_FLOOR_PICKUP);
    public static LimbSetpoint SPEAKER_SCORING_ARM_UP = new LimbSetpoint("Speaker Scoring", ElbowConstants.DEGREES_FLOOR_PICKUP, WristConstants.DEGREES_FLOOR_PICKUP);
    public static LimbSetpoint DEFENDED_SPEAKER_SCORING = new LimbSetpoint("Defended Speaker Scoring", ElbowConstants.DEGREES_START_CONFIG, WristConstants.DEGREES_START_CONFIG);
    public static LimbSetpoint AMP_SCORING = new LimbSetpoint("Amp Scoring", ElbowConstants.DEGREES_AMP_SCORE, WristConstants.DEGREES_AMP_SCORE);
    public static LimbSetpoint TRAP_SCORING = new LimbSetpoint("Trap Scoring", ElbowConstants.DEGREES_TRAP_SCORE, WristConstants.DEGREES_TRAP_SCORE);
    public static LimbSetpoint AMP_AND_SPEAKER_SOURCE_INTAKE = new LimbSetpoint("DualSource Intake", ElbowConstants.DEGREES_SOURCE_LOAD, WristConstants.DEGREES_SOURCE_LOAD);
    public static LimbSetpoint TRAP_SOURCE_INTAKE = new LimbSetpoint("TrapSource Intake", ElbowConstants.DEGREES_TRAP_LOAD_FROM_SOURCE, WristConstants.DEGREES_TRAP_LOAD_FROM_SOURCE);


    private String _name;
    private double _elbowRotationDegrees;
    private double _wristRotationDegrees;

    private LimbSetpoint(String name, double elbowRotationDegrees, double wristRotationDegrees) {
        _name = name;
        _elbowRotationDegrees = elbowRotationDegrees;
        _wristRotationDegrees = wristRotationDegrees * -1;
    }

    public String getName() {
        return _name;
    }

    public double getElbowRotationDegrees() {
        return _elbowRotationDegrees;
    }

    public double getWristRotationDegrees() {
        return _elbowRotationDegrees;
    }
    
    public double getElbowRotationPosition() {
        return ElbowSubsystem.getPositionFromRadians(Math.toRadians(_elbowRotationDegrees));
    }

    public double getWristRotationPosition() {
        return WristSubsystem.getPositionFromRadians(Math.toRadians(_wristRotationDegrees));
    }
}
