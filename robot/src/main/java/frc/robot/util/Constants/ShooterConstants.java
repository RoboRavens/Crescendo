package frc.robot.util.Constants;

public class ShooterConstants {
    
    public static final double lkP = 0.11;
    public static final double lkI = 0;
    public static final double lkD = 0;
    public static final double lmaxRPM = 240; // shooter wheel rotations per second
    public static final double lShooterVelocityPercentage = .80;

    public static final double rkP = 0.11;
    public static final double rkI = 0;
    public static final double rkD = 0;
    public static final double rmaxRPM = 210; // shooter wheel rotations per second
    public static final double rShooterVelocityPercentage = -.75;

    public static final double INITIAL_NOTE_SPEED = 20; // in m/s
    public static final double GRAVITY_ACCELERATION = 9.81;
    public static final double SPEAKER_HEIGHT_METERS = 2.047801077;

    public static final double MAX_SHOOTER_SPEED = 5;

    public static final double[][] SHOOTER_ANGLE_PAIRS_UP ={
        {0,90},
        {5,75},
        {10,60},
        {15,45}
    };

    public static final double[][] SHOOTER_ANGLE_PAIRS_DOWN ={
        {0.9144, 6},
        {1.181, 19.7},
        {2.032, 25.7},
        {2.3368, 31.4},
    };

    public static final double SHOOTER_STOP_DELAY = 1;
}
