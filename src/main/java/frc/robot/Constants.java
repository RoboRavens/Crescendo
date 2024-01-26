package frc.robot;

public class Constants {
    public static final double ARM_STARTING_DEGREES = 0.0;
    public static final double WRIST_STARTING_DEGREES = 0.0;

    public static final double kSlotIdx = 0;
    public static final double kPIDLoopIdx = 0;


    public static final double kTimeoutMs = 0;

    public static final double ARM_ROTATION_MAXIMUM_ENCODER_UNITS = ;
    public static final double WRIST_ROTATION_MAXIMUM_ENCODER_UNITS = ;

    public static final double ARM_ROTATION_TIMEOUT_ENCODER_TICKS_PER_SECOND = ;
    public static final double WRIST_ROTATION_TIMEOUT_ENCODER_TICKS_PER_SECOND = ;

    //find and add wrist
    public static final double ARM_TIMEOUT_BASE_VALUE = ;

    public static final double ARM_DEGREES_TO_ENCODER_UNITS = ;

    public static final Gains armRotationGains = new Gains(0.00000000000001, 0.00000000000001, 0.00000000000001, 0.00000000000001, 1, 0.00000000000001);
    public static final Gains wristRotationGains = new Gains(0.00000000000001, 0.00000000000001, 0.00000000000001, 0.00000000000001, 1, 0.00000000000001);

}
