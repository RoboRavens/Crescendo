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
    public static LimbSetpoint SPEAKER_SCORING = new LimbSetpoint("Speaker Scoring", ElbowConstants.DEGREES_SPEAKER_SCORING, WristConstants.DEGREES_SPEAKER_SCORING);
    public static LimbSetpoint DEFENDED_SPEAKER_SCORING = new LimbSetpoint("Defended Speaker Scoring", ElbowConstants.DEGREES_START_CONFIG, WristConstants.DEGREES_DEFENDED_SCORING);
    public static LimbSetpoint AMP_SCORING = new LimbSetpoint("Amp Scoring", ElbowConstants.DEGREES_AMP_SCORE, WristConstants.DEGREES_AMP_SCORE);
    public static LimbSetpoint TRAP_SCORING = new LimbSetpoint("Trap Scoring", ElbowConstants.DEGREES_TRAP_SCORE, WristConstants.DEGREES_TRAP_SCORE);
    public static LimbSetpoint SOURCE_INTAKE = new LimbSetpoint("DualSource Intake", ElbowConstants.DEGREES_SOURCE_LOAD, WristConstants.DEGREES_SOURCE_LOAD);
    public static LimbSetpoint TRAP_SOURCE_INTAKE = new LimbSetpoint("TrapSource Intake", ElbowConstants.DEGREES_TRAP_LOAD_FROM_SOURCE, WristConstants.DEGREES_TRAP_LOAD_FROM_SOURCE);
    public static LimbSetpoint SIX_NOTE_AUTO_GN_1_AND_2_SCORING_SETPOINT = new LimbSetpoint("Six Note Auto GN 1 And 2 Scoring Setpoint", ElbowConstants.DEGREES_FLOOR_PICKUP, WristConstants.DEGREES_SIX_NOTE_AUTO_GN_1_AND_2);
    public static LimbSetpoint SIX_NOTE_AUTO_GN_3_SCORING_SETPOINT = new LimbSetpoint("Six Note Auto GN 3 Scoring Setpoint", ElbowConstants.DEGREES_FLOOR_PICKUP, WristConstants.DEGREES_SIX_NOTE_AUTO_GN_3);
    public static LimbSetpoint SIX_NOTE_AUTO_GN_4_and_5_SCORING_SETPOINT = new LimbSetpoint("Six Note Auto GN 4 and 5 Scoring Setpoint", ElbowConstants.DEGREES_FLOOR_PICKUP, WristConstants.DEGREES_SIX_NOTE_AUTO_GN_4_AND_5);
    public static LimbSetpoint SIX_NOTE_AUTO_PRELOAD_SCORING_SETPOINT = new LimbSetpoint("Six Note Auto Preload Scoring Setpoint", ElbowConstants.DEGREES_FLOOR_PICKUP, WristConstants.DEGREES_SIX_NOTE_PRELOAD);
    public static LimbSetpoint START_CONFIG_UP = new LimbSetpoint("Start Config Up", ElbowConstants.DEGREES_START_CONFIG_UP, WristConstants.DEGREES_DEFENDED_SCORING);    
    public static LimbSetpoint SOUTH_CENTER_PRELOAD = new LimbSetpoint("South Center Preload", ElbowConstants.DEGREES_SOUTH_CENTER_PRELOAD, WristConstants.DEGREES_DEFENDED_SCORING);  
    public static LimbSetpoint PODIUM_SCORING = new LimbSetpoint("Podium Scoring", ElbowConstants.DEGREES_FLOOR_PICKUP, WristConstants.DEGREES_PODIUM_SCORE);
    public static LimbSetpoint STARTING_LINE_SCORING = new LimbSetpoint("Starting Line Scoring", ElbowConstants.DEGREES_FLOOR_PICKUP, WristConstants.DEGREES_STARTING_LINE_SCORE);

    private String _name;
    private double _elbowRotationDegrees;
    private double _wristRotationDegrees;

    public LimbSetpoint(String name, double elbowRotationDegrees, double wristRotationDegrees) {
        _name = name;
        _elbowRotationDegrees = elbowRotationDegrees;
        _wristRotationDegrees = wristRotationDegrees;
    }

    public String getName() {
        return _name;
    }

    public double getElbowRotationDegrees() {
        return _elbowRotationDegrees;
    }

    public double getWristRotationDegrees() {
        return _wristRotationDegrees;
    }
    
    public double getElbowRotationPosition() {
        return ElbowSubsystem.getPositionFromRadians(Math.toRadians(_elbowRotationDegrees));
    }

    public double getWristRotationPosition() {
        return WristSubsystem.getPositionFromRadians(Math.toRadians(_wristRotationDegrees));
    }
}
