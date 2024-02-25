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
}
