package frc.robot.util.Constants;

public class ShooterConstants {
    
    public static final double lkP = 0.11;
    public static final double lkI = 0;
    public static final double lkD = 0;
    public static final double lkS = 0.05;
    public static final double lkV = 0.12;
    public static final double lkMaxOutput = 1;
    public static final double lkMinOutput = -1;
    public static final double lmaxRPM = 5700;
    public static final double lShooterVelocityPercentage = 80;

    public static final double rkP = 0.11;
    public static final double rkI = 0;
    public static final double rkD = 0;
    public static final double rkS = 0.05;
    public static final double rkV = 0.12;
    public static final double rkMaxOutput = 1;
    public static final double rkMinOutput = -1;
    public static final double rmaxRPM = 5700;
    public static final double rShooterVelocityPercentage = 80;

    public static final double INITIAL_NOTE_SPEED = 20; // in m/s
    public static final double GRAVITY_ACCELERATION = 9.81;

    public static final double MAX_SHOOTER_SPEED = 5;

    public static final double[][] SHOOTER_ANGLE_PAIRS_UP ={
        {0,90},
        {5,75},
        {10,60},
        {15,45}
    };

    public static final double[][] SHOOTER_ANGLE_PAIRS_DOWN ={
        {0,90},
        {5,75},
        {10,60},
        {15,45}
    };

    public static final double SPEAKER_HEIGHT_METERS = 2.115;
    public static final double SHOOTER_DOWN_HEIGHT_METERS = 0.418;
    public static final double SHOOTER_UP_HEIGHT_METERS = 0.983;
    public static final double SHOOTER_STOP_DELAY = 1;
    public static final double LEFT_FEED_FORWARD = 0.5;
    public static final double RIGHT_FEED_FORWARD = 0.5;
}
