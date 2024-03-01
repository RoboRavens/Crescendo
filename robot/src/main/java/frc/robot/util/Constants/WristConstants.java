package frc.robot.util.Constants;

import com.ctre.phoenix6.configs.Slot0Configs;

import frc.robot.Gains;

public class WristConstants {
    public static final Gains WRIST_PID = new Gains(1, 0.000, 0.0000, 0.0, 0, 1);

	public static final double MOTOR_POWER_FEEDFORWARD_AT_HORIZONTAL = 0;
    public static final double MOTOR_POWER_DIRECTION_TO_GO_UP_FROM_HORIZONTAL = 0;
    
    public static final double ENCODER_POSITION_AT_GROUND_PICKUP = 0;
    public static final int ENCODER_POSITION_AT_VERTICAL = 0;
    public static final int ENCODER_POSITION_AT_HORIZONTAL = 0;

    public static double ANGLE_FLOOR_PICKUP = 0.0;
    public static double ANGLE_AMP_SCORE = -20;
    public static double ANGLE_TRAP_LOAD_FROM_SOURCE = -20;
    public static double ANGLE_SOURCE_LOAD = -40;
    public static double ANGLE_TRAP_SCORE = -40;
    public static double START_CONFIG = 46;

    public static Slot0Configs getSlot0Configs() {
        var slot0Config = new Slot0Configs();
        slot0Config.kP = Constants.ELBOW_PID.kP;
        slot0Config.kI = Constants.ELBOW_PID.kI;
        slot0Config.kD = Constants.ELBOW_PID.kD;
        return slot0Config;
    }
}
