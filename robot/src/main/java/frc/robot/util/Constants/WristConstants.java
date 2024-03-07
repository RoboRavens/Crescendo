package frc.robot.util.Constants;

import com.ctre.phoenix6.configs.Slot0Configs;

public class WristConstants {
	// public static final double MOTOR_POWER_FEEDFORWARD_AT_HORIZONTAL = -0.079; original, save for later
    public static final double MOTOR_POWER_FEEDFORWARD_AT_HORIZONTAL = -.16;
    public static final double MOTOR_POWER_DIRECTION_TO_GO_UP_FROM_HORIZONTAL = 0;
    
    public static final double ENCODER_POSITION_AT_ROBOT_START = 0;
    public static final double ENCODER_POSITION_45_FROM_ROBOT_START = -3.049348;

    public static final double IS_AT_SETPOINT_BUFFER = .2;

    // these constants show degrees from floor pickup and are relative to the arm (not relative to the field)
    // these match what is shown in onshape, but may be adjusted based on testing
    // be careful, these match onshape signs, the sign shown on the dashboard is inverted
    // if dashboard shows 35, then put -35 here
    public static double DEGREES_FLOOR_PICKUP = 0.0;
    public static double DEGREES_AMP_SCORE = -7.580386;
    public static double DEGREES_TRAP_LOAD_FROM_SOURCE = -20;
    public static double DEGREES_SOURCE_LOAD = -54.979414;
    public static double DEGREES_TRAP_SCORE = -40;
    public static double DEGREES_START_CONFIG = 46;
    // TODO: Update these target values
    public static double DEGREES_SIX_NOTE_AUTO_GN_1_AND_2 = 0;
    public static double DEGREES_SIX_NOTE_AUTO_GN_3 = 0;
    public static double DEGREES_SIX_NOTE_AUTO_GN_4_AND_5 = 0;
    public static double DEGREES_SIX_NOTE_PRELOAD = 0;


    public static Slot0Configs getSlot0Configs() {
        var slot0Config = new Slot0Configs();
        slot0Config.kP = 5;
        slot0Config.kI = 0;
        slot0Config.kD = 0;
        return slot0Config;
    }
}
