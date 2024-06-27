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
        {22.5, -20}, // 47 inches, -19.619653 wrist angle degrees
        {18.9, -15}, // 53 inches, -14.810095 wrist angle degrees
        {16.2, 4}, // 59 inches, 3.769056 wrist angle degrees
        {13.3, 6}, // 65 inches, 5.794335 wrist angle degrees
        {10.8, 7}, // 72 inches, 6.781995 wrist angle degrees
        {8.6, 9},// 78 inches, 8.755394 wrist angle degrees
        {7.5, 9.75}, // 84 inches, 9.529765 wrist angle degrees
        {5.6, 11.75}, // 90 inches, 11.503163 wrist angle degrees
        {4.4, 12.5}, // 96 inches, 12.246791 wrist angle degrees
        {3, 14} // 102 inches, 13.766711 wrist angle degrees
        /*
        {22.5, -1}, // 47 inches, 16.266845 wrist angle degrees
        {21.00, -1}, // 47 inches, 16.266845 wrist angle degrees
        {14.75, 3.5}, // 60 inches, 20.38 wrist angle degrees
        {10.53, 6}, // 72 inches, 23.74 wrist angle degrees
        {5.34, 8.5}, // 89 inches, 27.608601 wrist angle degrees
        {2.05, 10.25}, // 105 inches, 29.35 wrist angle degrees
        {-0.27, 11.5}, // 120 inches, 30.07 wrist angle degrees
        {-1.47, 12.25}, // 127 inches, 32.119363 wrist angle degrees
        {-4.47, 13}, // 151 inches, 33.27 wrist angle degrees
        */
    };

    public static final double[][] SHOOTER_ANGLE_FROM_TY_UP = {
        {2.969733, -33.9}, // 102 inches, -33.892384 wrist angle degrees
        {4.259582, -32}, // 96 inches, -32.236335 wrist angle degrees
        {5.647132, -33.5}, // 90 inches, -33.715904 wrist angle degrees
        {6.9, -35.25}, // 84 inches, -35.454877 wrist degree angles
        {8.91, -37}, // 78 inches, -37.195772 wrist angle degrees
        {10.908, -37}, // 72 inches, -37.23966 wrist angle degrees
        {13.342574, -38}, // 66 inches, -38.218019 wrist angle degrees
        {15.044, -39}, // 60 inches, -39.224894 wrist angle degrees
        {18.107405, -40.5}, // 54 inches, -40.717913 wrist angle degrees
        {21.912, -43}, // 47 inches, -43.240865 wrist angle degrees
        
    };

    public static final double SHOOTER_STOP_DELAY = 1;
}
