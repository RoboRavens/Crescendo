package frc.robot.util.Constants;

public class Constants {
    // Limelight
    public static final double STATE_STANDARD_DEVIATION = .1;
    public static final double STARTING_VISION_STANDARD_DEVIATION = .9;
    public static final double MINIMUM_VISION_STANDARD_DEVIATION = .05;
    public static final double TX_CLOSE_TO_SPEAKER_THRESHOLD_DEGREES = 5;

    // Control
    public static final double JOYSTICK_DEADBAND = .1;
    public static final double DRIVE_MAX_TURN_RADIANS_PER_SECOND = 7;

    // DRIVETRAIN
    public static final double SLEW_FRAMES_TO_MAX_X_VELOCITY = 10; // forward-back
    public static final double SLEW_FRAMES_TO_MAX_Y_VELOCITY = 10; // left-right
    public static final double SLEW_FRAMES_TO_MAX_ANGULAR_VELOCITY = 10; // turning
    public static final double DRIVE_HOLD_ANGLE_TIMEOUT_SECONDS = .33;
    public static final double DRIVETRAIN_HOLD_POSITION_TIMER_THRESHOLD = .5;
    public static final double MAXIMUM_OFFSET_FROM_CENTER_OF_SPEAKER_METERS = .5;

    // Move Elbow and Wrist
    public static final double MOVE_ELBOW_UP_MANUAL_POWER = -0.1; // Elbow up
    public static final double MOVE_ELBOW_DOWN_MANUAL_POWER = 0.1; // Elbow down
    public static final Double MOVE_WRIST_UP_MANUAL_POWER = -0.1; // Wrist up
    public static final Double MOVE_WRIST_DOWN_MANUAL_POWER = 0.1; // Wrist down
    public static final int LED_MODES_LBUMPER_PRESS = 6;
}
