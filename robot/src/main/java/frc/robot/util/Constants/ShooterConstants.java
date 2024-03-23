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
        {21.05, 18}, // 47 inches, 16.266845 wrist angle degrees
        {14.75, 23.25}, // 60 inches, 20.38 wrist angle degrees
        {10.53, 27.5}, // 72 inches, 23.74 wrist angle degrees
        {5.34, 31.25}, // 89 inches, 27.608601 wrist angle degrees
        {2.05, 33}, // 105 inches, 29.35 wrist angle degrees
        {-0.27, 31.75}, // 120 inches, 30.07 wrist angle degrees
        {-1.47, 33.75}, // 127 inches, 32.119363 wrist angle degrees
        {-4.47, 35}, // 151 inches, 33.27 wrist angle degrees
    };

    public static final double[][] SHOOTER_ANGLE_FROM_TY_UP = {
        {22, -40}, // 47 inches, -41.659696 wrist angle degrees
        {14.2, -32}, // 63 inches, -33.646969 wrist angle degrees
        {7.57, -31}, // 81 inches, -32.3 wrist angle degrees
        {3.7, -28}, // 98 inches, -29.47 wrist angle degrees
        {0.58, -26}, // 115 inches, -27.378019 wrist angle degrees
        {-1.91, -24.5}, // 133 inches, -25.951292 wrist angle degrees
        {-4.3, -23.5}, // 151 inches, -24.856027 wrist angle degrees
    };

    public static final double SHOOTER_STOP_DELAY = 1;
}
