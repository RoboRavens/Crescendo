package frc.robot.util.Constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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

    public static final double SWERVE_CONTROLLER_X_KP = 2;
    public static final double SWERVE_CONTROLLER_Y_KP = 2;
    public static final double SWERVE_CONTROLLER_ANGLE_KP = 4;

    public static final double COORDINATE_MATCHES_MARGIN_METERS = Units.inchesToMeters(2);
    public static final double ROTATION_MATCHES_MARGIN_DEGREES = 3.0;
    public static final double TRAJECTORY_CONFIG_MAX_VELOCITY_METERS_PER_SECOND = 1.5;
    public static final double TRAJECTORY_CONFIG_MAX_ACCELERATION_METERS_PER_SECOND = .6;
    public static final double TRAJECTORY_CONFIG_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
    public static final double TRAJECTORY_CONFIG_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND = Math.PI * .75;
    
      // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints SWERVE_CONTROLLER_ANGULAR_CONSTRAINTS =
        new TrapezoidProfile.Constraints(
            TRAJECTORY_CONFIG_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, TRAJECTORY_CONFIG_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND);

}
