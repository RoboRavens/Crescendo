package frc.robot.util.Constants;

public class ShooterConstants {
    // every 1 % power is roughly 150 RPM or 2.5 RPS
    public static final double lkP = 0.11;
    public static final double lkI = 0;
    public static final double lkD = 0;
    public static final double FF_FOR_TARGET_LEFT = .37;
    public static final double TARGET_RPS_LEFT = 96; // lmaxRPM * lShooterVelocityPercentage;
    public static final double ACTUAL_PID_RPS_FOR_SOME_REASON_LEFT = 71;

    // right shooter is slightly stronger, every 1 % power is roughly 150 RPM or 2.5 RPS
    public static final double rkP = 0.11;
    public static final double rkI = 0;
    public static final double rkD = 0;
    public static final double FF_FOR_TARGET_RIGHT = .56;
    public static final double TARGET_RPS_RIGHT = 144; // rmaxRPM * rShooterVelocityPercentage;
    public static final double ACTUAL_PID_RPS_FOR_SOME_REASON_RIGHT = 110;

    public static final double IS_AT_TARGET_SPEED_BUFFER = 10;

    public static final double INITIAL_NOTE_SPEED = 20; // in m/s
    public static final double GRAVITY_ACCELERATION = 9.81;
    public static final double SPEAKER_HEIGHT_METERS = 2.047801077;

    public static final double MAX_SHOOTER_SPEED = 5;
    public static final double REVERSE_SPEED_PERCENTAGE = -0.3;

    public static final double[][] SHOOTER_ANGLE_PAIRS_UP ={
        {0,90},
        {5,75},
        {10,60},
        {15,45}
    };

    public static final double[][] SHOOTER_ANGLE_PAIRS_DOWN ={
        {0.9144, 6},
        {1.181, 19.7},
        {1.962, 16}, // start line; 77.25 inches
        {2.3368, 34.4}, // podium + 3 degrees
    };

    public static final double[][] SHOOTER_ANGLE_FROM_TY_DOWN = {
        {22.175, 0}, // 11 inches
        {15.1, 12}, // 24 inches
        {10.8, 17}, // 36 inches
        {5.693221, 20}, // 53 inches
        {2.45, 22}, // 69 inches
        {-1.310349, 24}, // 91 inches
    };

    public static final double[][] SHOOTER_ANGLE_FROM_TY_UP = {
        {50, 0},
        {-50, 0},
    };

    public static final double SHOOTER_STOP_DELAY = 1;
}
