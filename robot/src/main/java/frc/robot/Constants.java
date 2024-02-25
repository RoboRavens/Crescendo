package frc.robot;

public class Constants {
    // Limelight
    public static final double STATE_STANDARD_DEVIATION = .1;
    public static final double STARTING_VISION_STANDARD_DEVIATION = .9;
    public static final double MINIMUM_VISION_STANDARD_DEVIATION = .05;

    // Control
    public static final double JOYSTICK_DEADBAND = .1;
    public static final double DRIVE_MAX_TURN_RADIANS_PER_SECOND = 3;

    // DRIVETRAIN SLEW
    public static final double SLEW_FRAMES_TO_MAX_X_VELOCITY = 10; // forward-back
    public static final double SLEW_FRAMES_TO_MAX_Y_VELOCITY = 10; // left-right
    public static final double SLEW_FRAMES_TO_MAX_ANGULAR_VELOCITY = 10; // turning

    public static final Gains ELBOW_PID = new Gains(1, 0.000, 0.0000, 0.0, 0, 1);
}
