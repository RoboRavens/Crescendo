package frc.robot.util.Constants;

import com.ctre.phoenix6.configs.Slot0Configs;

import frc.robot.Gains;

public class WristConstants {
    public static final Gains WRIST_PID = new Gains(1, 0.000, 0.0000, 0.0, 0, 1);

	public static final double MOTOR_POWER_FEEDFORWARD_AT_HORIZONTAL = -0.079;
    public static final double MOTOR_POWER_DIRECTION_TO_GO_UP_FROM_HORIZONTAL = 0;
    
    public static final double ENCODER_POSITION_AT_ROBOT_START = 0;
    public static final double ENCODER_POSITION_45_FROM_ROBOT_START = -3.049348;

    // these constants show degrees from floor pickup and are relative to the arm (not relative to the field)
    // these match what is shown in onshape, but may be adjusted based on testing
    public static double DEGREES_FLOOR_PICKUP = 0.0;
    public static double DEGREES_AMP_SCORE = -20;
    public static double DEGREES_TRAP_LOAD_FROM_SOURCE = -20;
    public static double DEGREES_SOURCE_LOAD = -40;
    public static double DEGREES_TRAP_SCORE = -40;
    public static double DEGREES_START_CONFIG = 46;

    public static Slot0Configs getSlot0Configs() {
        var slot0Config = new Slot0Configs();
        slot0Config.kP = Constants.ELBOW_PID.kP;
        slot0Config.kI = Constants.ELBOW_PID.kI;
        slot0Config.kD = Constants.ELBOW_PID.kD;
        return slot0Config;
    }
}
