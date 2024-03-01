package frc.robot.util.Constants;

import frc.robot.Gains;

public class WristConstants {
    public static final Gains WRIST_PID = new Gains(1, 0.000, 0.0000, 0.0, 0, 1);

    public static double ANGLE_FLOOR_PICKUP = 0.0;
    public static double ANGLE_AMP_SCORE = -20;
    public static double ANGLE_TRAP_LOAD_FROM_SOURCE = -20;
    public static double ANGLE_SOURCE_LOAD = -40;
    public static double ANGLE_TRAP_SCORE = -40;
    public static double START_CONFIG = 46;
}
