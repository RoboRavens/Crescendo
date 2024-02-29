package frc.util;

public class StateManagement {
    // Both the button panel and the teleop dashboard will update these target states (initialized in Robot.java)
    // The Robot.java target states may be used anywhere in code to fire other commands
    public enum ScoringTargetState {
        SPEAKER,
        AMP,
        TRAP,
        CLIMB
    }

    public enum IntakeTargetState {
        GROUND,
        SOURCE,
        TRAP_SOURCE
    }

    public enum LEDSignalTargetState {
        CO_OP_SIGNAL,
        AMP_SIGNAL,
        NONE
    }

    public enum TrapSourceLaneTargetState {
        LEFT,
        CENTER,
        RIGHT
    }

    public enum ArmUpTargetState {
        FREE,
        UP
    }

    public enum ShooterRevTargetState {
        ON,
        OFF
    }

    // From the driver station POV
    public enum ClimbPositionTargetState {
        LEFT_FAR,
        LEFT_CENTER,
        LEFT_CLOSE,
        RIGHT_CLOSE,
        RIGHT_CENTER,
        RIGHT_FAR,
        OPPOSITE_RIGHT,
        OPPOSITE_CENTER,
        OPPOSITE_LEFT
    }
    // TODO: Set this state
    public enum ZoneState {
        NONE,
        ALLIANCE_WING,
        ALLIANCE_STAGE,
        ALLIANCE_SOURCE,
        ALLIANCE_AMP,
        NEUTRAL,
        OPPONENT_WING,
        OPPONENT_STAGE,
        OPPONENT_SOURCE,
        OPPONENT_AMP,
    }
    // TODO: Set this state
    public enum LoadState {
        LOADED,
        TRAP_LOADED,
        EMPTY
    }

    public enum OverallState {
        EMPTY_TRANSIT, // When the robot does not have a note
        SEEKING_NOTE, // When the robot does not have a note but has a target scoring state that requires a note
        LOADED_TRANSIT,
        LOADING, // When the robot detects a note and has a target scoring state that requires a note
        SCORING,
        PREPARING_TO_SCORE,
    }

    public enum DrivetrainState {
        FREEHAND,
        ROBOT_ALIGN
    }
    // TODO: Set this state
    public enum LimelightDetectsNoteState {
        IN_RANGE,
        NO_NOTE
    }
}
