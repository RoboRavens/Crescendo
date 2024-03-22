package frc.robot.util.Constants;

import com.ctre.phoenix6.configs.Slot0Configs;

public class WristConstants {
	// public static final double MOTOR_POWER_FEEDFORWARD_AT_HORIZONTAL = -0.079; original, save for later
    public static final double MOTOR_POWER_FEEDFORWARD_AT_HORIZONTAL = -.08;
    public static final double DEGREES_OFFSET_TO_VERTICAL = 40;
    public static final double MOTOR_POWER_DIRECTION_TO_GO_UP_FROM_HORIZONTAL = 0;
    
    public static final double ENCODER_POSITION_AT_FLOOR_PICKUP = 0;
    public static final double ENCODER_POSITION_45_FROM_FLOOR_PICKUP = -3.049348;
    public static final double ENCODER_POSITION_AT_STARTUP = 4.292480;

    public static final double IS_AT_SETPOINT_BUFFER = .2;

    public static final double WRIST_DEGREES_PER_INCREMENT = 2.5;

    // these constants show degrees from floor pickup and are relative to the arm (not relative to the field)
    // values from onshape need to have their sign inverted
    // as degrees get bigger (right d-pad), the shooting angle becomes less (note hits the speaker at lower height)
    public static final double DEGREES_FLOOR_PICKUP = 0.0;
    public static final double DEGREES_AMP_SCORE = -16.2;
    public static final double DEGREES_TRAP_LOAD_FROM_SOURCE = 20;
    public static final double DEGREES_SOURCE_LOAD = 50;
    public static final double DEGREES_TRAP_SCORE = 40;
    public static final double DEGREES_DEFENDED_SCORING = -39;
    public static final double DEGREES_SPEAKER_SCORING = 0.0;

    // AUTO MODE ANGLES
    public static final double DEGREES_SIX_NOTE_AUTO_GN_1_AND_2 = 0;
    public static final double DEGREES_SIX_NOTE_AUTO_GN_3 = 31;
    public static final double DEGREES_SIX_NOTE_AUTO_GN_4_AND_5 = 21;
    public static final double DEGREES_SIX_NOTE_PRELOAD = 0;
    public static final double DEGREES_SOURCE_SIDE_AUTO_WING_SHOT = 21; // TODO: find the correct number of degrees

    // distance in meters from speaker to back bumper
    public static final double DISTANCE_METERS_STARTING_LINE = 2.032;
    public static final double DISTANCE_METERS_PODIUM = 2.3368;

    public static final double JOYSTICK_CONTROL_SCALING_FACTOR = .1;

    public static final double DEGREES_DOWNWARD_MOTION_BUFFER = 15;


    public static Slot0Configs getSlot0Configs() {
        var slot0Config = new Slot0Configs();
        slot0Config.kP = 5;
        slot0Config.kI = 0;
        slot0Config.kD = .25;
        return slot0Config;
    }
}
