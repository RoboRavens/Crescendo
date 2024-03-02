package frc.robot.util.Constants;

import frc.robot.Gains;

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

    // Move Elbow and Wrist
    public static final double MOVE_ELBOW_UP_MANUAL_POWER = -0.1; // Elbow up
    public static final double MOVE_ELBOW_DOWN_MANUAL_POWER = 0.1; // Elbow down
    public static final Double MOVE_WRIST_UP_MANUAL_POWER = -0.1; // Wrist up
    public static final Double MOVE_WRIST_DOWN_MANUAL_POWER = 0.1; // Wrist down

    // LEDs
    public static final int TOTAL_LEDS_STRIP_LENGTH = 111;
    public static final int END_OF_FIRST_LEDS_SECTION = 28;
    public static final int END_OF_SECOND_LEDS_SECTION = 56;
    public static final int END_OF_THIRD_LEDS_SECTION = 84;
    public static final int END_OF_FOURTH_LEDS_SECTION = TOTAL_LEDS_STRIP_LENGTH;


}
