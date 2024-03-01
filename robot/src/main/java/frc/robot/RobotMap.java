package frc.robot;

public class RobotMap {
    // CONTROLLERS
    public static final int DRIVE_CONTROLLER_PORT = 0;
    public static final int OVERRIDES_CONTROLLER_PORT = 1;
    public static final int BUTTONS_CONTROLLER_PORT = 2;

    // DRIVETRAIN
    // start at 10
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.4445; // The left-to-right distance between the drivetrain wheels
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5461; // The front-to-back distance between the drivetrain wheels.

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 16; // Updated 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 15; // Updated
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 21; // Updated

    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = Math.toRadians(220);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 17; // Updated
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 11; // Updated
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 24; // Updated
 
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(35);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 10; // Updated
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 14; // Updated
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 22; // Updated

    public static final double BACK_LEFT_MODULE_STEER_OFFSET = Math.toRadians(225);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 13; // Updated
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 12; // Updated
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 23; // Updated
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(170); 
    
    // These are Shooter Motors
    public static final int SHOOTER_LEFT_MOTOR_CAN_ID = 6;
    public static final int SHOOTER_RIGHT_MOTOR_CAN_ID = 7;

    // Intake
    public static final int INTAKE_MOTOR_CAN_ID = 8;
    
    //needs can IDs these are placeholders
    public static final int ELBOW_ROTATION_MOTOR = 1;
    public static final int ELBOW_ROTATION_FOLLOWER_MOTOR = 2;
    public static final int WRIST_ROTATION_MOTOR = 3;
     
    // These are DIO Ports
    public static final int ELBOW_FORWARD_LIMIT_DIO = 0;
    public static final int WRIST_FORWARD_LIMIT_DIO = 0; //?
    public static final int INTAKE_INTAKE_SENSOR_DIO_PORT = 1;
    public static final int INTAKE_TRAP_SENSOR_DIO_PORT = 2;
    public static final int SHOOTER_PIECE_SENSOR = 3;
}
