package frc.robot.subsystems;

public class ShooterConstants {
    
    public static final double lkP = 6e-5;
    public static final double lkI = 0;
    public static final double lkD = 0;
    public static final double lkIz = 0;
    public static final double lkFF = 0.000015;
    public static final double lkMaxOutput = 1;
    public static final double lkMinOutput = -1;
    public static final double lmaxRPM = 5700;
    public static final double lShooterVelocityPercentage = 80;

    public static final double rkP = 6e-5;
    public static final double rkI = 0;
    public static final double rkD = 0;
    public static final double rkIz = 0;
    public static final double rkFF = 0.000015;
    public static final double rkMaxOutput = 1;
    public static final double rkMinOutput = -1;
    public static final double rmaxRPM = 5700;
    public static final double rShooterVelocityPercentage = 80;

    public static final double INITIAL_NOTE_SPEED = 20; // in m/s
    public static final double GRAVITY_ACCELERATION = 9.81;
    public static final double SPEAKER_HEIGHT_METERS = 2.047801077;

    public static final double MAX_SHOOTER_SPEED = 5;

    public static final double[][] SHOOTER_ANGLE_PAIRS ={
        {0,90},
        {5,75},
        {10,60},
        {15,45}
    };

    public static final double SHOOTER_STOP_DELAY = 1;

}
