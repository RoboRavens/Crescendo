package frc.robot;

import frc.robot.util.arm.LimbSetpoint;

public class Constants {

    //?? double check value
    public static final double COUNTS_PER_REVOLUTION = 4096;
        public static final double ELBOW_DEGREES_TO_ENCODER_UNITS = COUNTS_PER_REVOLUTION / 360;
    public static final double WRIST_DEGREES_TO_ENCODER_UNITS = COUNTS_PER_REVOLUTION / 360;

    //check with CAD for actual values
    public static final double WRIST_GROUND_ROTATION_ANGLE = 0;
    public static final double WRIST_SOURCE_ROTATION_ANGLE = 0;
    public static final double WRIST_UNDER_SPEAKER_ROTATION_ANGLE = 0;
    public static final double WRIST_TRAP_ROTATION_ANGLE = 0;
    public static final double WRIST_AMP_ROTATION_ANGLE = 0;
    public static final double WRIST_WING_SPEAKER_ANGLE = 0;
    public static final double WRIST_END_STAGE_SPEAKER_ANGLE = 0;    
    public static final double WRIST_MAXIMUM_ANGLE = 180;
    public static final double WRIST_MINIMUM_ANGLE = 0;

    //check with CAD for actual values
    public static final double ELBOW_GROUND_ROTATION_ANGLE = 0;
    public static final double ELBOW_SOURCE_ROTATION_ANGLE = 0;
    public static final double ELBOW_UNDER_SPEAKER_ROTATION_ANGLE = 0;
    public static final double ELBOW_TRAP_ROTATION_ANGLE = 0;
    public static final double ELBOW_AMP_ROTATION_ANGLE = 0;
    public static final double ELBOW_WING_SPEAKER_ANGLE = 0;
    public static final double ELBOW_END_STAGE_SPEAKER_ANGLE = 0;   
    public static final double ELBOW_MAXIMUM_ANGLE = 180;
    public static final double ELBOW_MINIMUM_ANGLE = 0;

    //native encoder units, none correct values
    public static final double WRIST_GROUND_INTAKE_SETPOINT = Math.round(WRIST_GROUND_ROTATION_ANGLE * WRIST_DEGREES_TO_ENCODER_UNITS);
    public static final double ELBOW_GROUND_INTAKE_SETPOINT = Math.round(ELBOW_GROUND_ROTATION_ANGLE * ELBOW_DEGREES_TO_ENCODER_UNITS);
    public static final double WRIST_SOURCE_INTAKE_SETPOINT = Math.round(WRIST_SOURCE_ROTATION_ANGLE * WRIST_DEGREES_TO_ENCODER_UNITS);
    public static final double ELBOW_SOURCE_INTAKE_SETPOINT = Math.round(ELBOW_SOURCE_ROTATION_ANGLE * ELBOW_DEGREES_TO_ENCODER_UNITS);
    public static final double WRIST_UNDER_SPEAKER_SCORING_SETPOINT = Math.round(WRIST_UNDER_SPEAKER_ROTATION_ANGLE * WRIST_DEGREES_TO_ENCODER_UNITS);
    public static final double ELBOW_UNDER_SPEAKER_SCORING_SETPOINT = Math.round(ELBOW_UNDER_SPEAKER_ROTATION_ANGLE * ELBOW_DEGREES_TO_ENCODER_UNITS);
    public static final double WRIST_TRAP_SCORING_SETPOINT = Math.round(WRIST_TRAP_ROTATION_ANGLE * WRIST_DEGREES_TO_ENCODER_UNITS);
    public static final double ELBOW_TRAP_SCORING_SETPOINT = Math.round(ELBOW_TRAP_ROTATION_ANGLE * ELBOW_DEGREES_TO_ENCODER_UNITS);
    public static final double WRIST_AMP_SCORING_SETPOINT = Math.round(WRIST_AMP_ROTATION_ANGLE * WRIST_DEGREES_TO_ENCODER_UNITS);
    public final static double ELBOW_AMP_SCORING_SETPOINT = Math.round(ELBOW_AMP_ROTATION_ANGLE * ELBOW_DEGREES_TO_ENCODER_UNITS);
    public final static double WRIST_WING_SPEAKER_SETPOINT = Math.round(WRIST_WING_SPEAKER_ANGLE * WRIST_DEGREES_TO_ENCODER_UNITS);
    public final static double WRIST_END_STAGE_SPEAKER_SETPOINT = Math.round(WRIST_END_STAGE_SPEAKER_ANGLE * ELBOW_DEGREES_TO_ENCODER_UNITS);
    public final static double ELBOW_WING_SPEAKER_SETPOINT = Math.round(ELBOW_WING_SPEAKER_ANGLE * WRIST_DEGREES_TO_ENCODER_UNITS);
    public final static double ELBOW_END_STAGE_SPEAKER_SETPOINT = Math.round(ELBOW_END_STAGE_SPEAKER_ANGLE * ELBOW_DEGREES_TO_ENCODER_UNITS);


    public static final LimbSetpoint WRIST_GROUND_INTAKE_COMMAND_SETPOINT = new LimbSetpoint("Wrist Ground Intake", WRIST_GROUND_INTAKE_SETPOINT);
    public static final LimbSetpoint ELBOW_GROUND_INTAKE_COMMAND_SETPOINT = new LimbSetpoint("Elbow Ground Intake", ELBOW_GROUND_INTAKE_SETPOINT);
    public static final LimbSetpoint WRIST_SOURCE_INTAKE_COMMAND_SETPOINT = new LimbSetpoint("Wrist Human IntaKE", WRIST_SOURCE_INTAKE_SETPOINT);
    public static final LimbSetpoint ELBOW_SOURCE_INTAKE_COMMAND_SETPOINT = new LimbSetpoint("Elbow Human IntaKE", ELBOW_SOURCE_INTAKE_SETPOINT);
    public static final LimbSetpoint WRIST_UNDER_SPEAKER_SCORING_COMMAND_SETPOINT = new LimbSetpoint("Wrist Speaker Scoring", WRIST_UNDER_SPEAKER_SCORING_SETPOINT);
    public static final LimbSetpoint ELBOW_UNDER_SPEAKER_SCORING_COMMAND_SETPOINT = new LimbSetpoint("Elbow Speaker Scoring", ELBOW_UNDER_SPEAKER_SCORING_SETPOINT);
    public static final LimbSetpoint WRIST_TRAP_SCORING_COMMAND_SETPOINT = new LimbSetpoint("Wrist Trap Scoring", WRIST_TRAP_SCORING_SETPOINT);
    public static final LimbSetpoint ELBOW_TRAP_SCORING_COMMAND_SETPOINT = new LimbSetpoint("Elbow Trap Scoring", ELBOW_TRAP_SCORING_SETPOINT );
    public static final LimbSetpoint WRIST_AMP_SCORING_COMMAND_SETPOINT = new LimbSetpoint("Wrist Amp Scoring", WRIST_AMP_SCORING_SETPOINT);
    public static final LimbSetpoint ELBOW_AMP_SCORING_COMMAND_SETPOINT = new LimbSetpoint("Elbow Amp Scoring", ELBOW_AMP_SCORING_SETPOINT);
    public static final LimbSetpoint WRIST_WING_SPEAKER_SCORING_COMMAND_SETPOINT = new LimbSetpoint("Wrist Wing Speaker Scoring", WRIST_WING_SPEAKER_SETPOINT);
    public static final LimbSetpoint ELBOW_WING_SPEAKER_SCORING_COMMAND_SETPOINT = new LimbSetpoint("Arm Wing Speaker Scoring", WRIST_END_STAGE_SPEAKER_SETPOINT);
    public static final LimbSetpoint WRIST_STAGE_END_SPEAKER_SCORING_COMMAND_SETPOINT = new LimbSetpoint("Wrist Stage End Speaker Scoring", ELBOW_WING_SPEAKER_SETPOINT);
    public static final LimbSetpoint ELBOW_STAGE_END_SPEAKER_SCORING_COMMAND_SETPOINT = new LimbSetpoint("Arm Stage End Speaker Scorin", ELBOW_END_STAGE_SPEAKER_SETPOINT);

    //??
    public static final double ELBOW_ROTATION_IS_AT_SETPOINT_MARGIN_ENCODER_TICKS = 500;
    public static final double WRIST_ROTATION_IS_AT_SETPOINT_MARGIN_ENCODER_TICKS = 500;

    //?? determine actual values
    public static final double ELBOW_ROTATION_VELOCITY = 0;
    public static final double ELBOW_ROTATION_ACCELERATION = 0;
    public static final double WRIST_ROTATION_VELOCITY = 0;
    public static final double WRIST_ROTATION_ACCELERATION = 0;

    //differ with starting position?? 
    public static final double ELBOW_STARTING_DEGREES = 0.0;
    public static final double WRIST_STARTING_DEGREES = 0.0;
 

    public static final double ELBOW_ROTATION_MAXIMUM_ENCODER_UNITS = ELBOW_MAXIMUM_ANGLE * ELBOW_DEGREES_TO_ENCODER_UNITS;
    public static final double ELBOW_ROTATION_MINIMUM_ENCODER_UNITS = ELBOW_MINIMUM_ANGLE * ELBOW_DEGREES_TO_ENCODER_UNITS;
    public static final double WRIST_ROTATION_MAXIMUM_ENCODER_UNITS = WRIST_MAXIMUM_ANGLE * WRIST_DEGREES_TO_ENCODER_UNITS;
    public static final double WRIST_ROTATION_MINIMUM_ENCODER_UNITS = WRIST_MINIMUM_ANGLE * WRIST_DEGREES_TO_ENCODER_UNITS;

    //?? determine correct values
    public static final double ELBOW_ROTATION_TIMEOUT_ENCODER_TICKS_PER_SECOND = 2048;
    public static final double WRIST_ROTATION_TIMEOUT_ENCODER_TICKS_PER_SECOND = 2048;

    //?? assign real values
    public static final double ELBOW_TIMEOUT_BASE_VALUE = 1;
    public static final double WRIST_TIMEOUT_BASE_VALUE = 1;

    //?? reconfirm values
    public static final double ELBOW_ROTATION_MANUAL_DEGREES_PER_SECOND = 90;
    public static final double ELBOW_ROTATION_MANUAL_NATIVE_UNITS_PER_TICK = ELBOW_ROTATION_MANUAL_DEGREES_PER_SECOND * ELBOW_DEGREES_TO_ENCODER_UNITS / 50;
    public static final double WRIST_ROTATION_MANUAL_DEGREES_PER_SECOND = 90;
    public static final double WRIST_ROTATION_MANUAL_NATIVE_UNITS_PER_TICK = WRIST_ROTATION_MANUAL_DEGREES_PER_SECOND * WRIST_DEGREES_TO_ENCODER_UNITS / 50;

    public static final Gains elbowRotationGains = new Gains(0.5, 0.001, 0.0000, 0.0, 0, 1);
    public static final Gains wristRotationGains = new Gains(0.5, 0.001, 0.0000, 0.0, 0, 1);

    public static final double WRIST_MANUAL_ROTATION_VOLTAGE = 0;
    public static final double ELBOW_MANUAL_ROTATION_VOLTAGE = 0;
}
