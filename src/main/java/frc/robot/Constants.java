package frc.robot;

import frc.robot.util.arm.ArmSetpoint;

public class Constants {

    public static final ArmSetpoint ARM_GROUND_INTAKE_COMMAND_SETPOINT = new ArmSetpoint("Ground Intake", WRIST_GROUND_INTAKE_SETPOINT, ARM_GROUND_INTAKE_SETPOINT);
    public static final double WRIST_GROUND_INTAKE_SETPOINT = ;
    public static final double ARM_GROUND_INTAKE_SETPOINT = ;

    public static final double ARM_ROTATION_IS_AT_SETPOINT_MARGIN_ENCODER_TICKS = ;
    public static final double WRIST_ROTATION_IS_AT_SETPOINT_MARGIN_ENCODER_TICKS = ;

    public static final double ARM_ROTATION_VELOCITY = ;
    public static final double ARM_ROTATION_ACCELERATION = ;
    public static final double WRIST_ROTATION_VELOCITY = ;
    public static final double WRIST_ROTATION_ACCELERATION = ;

    public static final double ARM_STARTING_DEGREES = 0.0;

    public static final double ARM_ROTATION_MAXIMUM_ENCODER_UNITS = ;
    public static final double WRIST_ROTATION_MAXIMUM_ENCODER_UNITS = ;

    public static final double ARM_ROTATION_TIMEOUT_ENCODER_TICKS_PER_SECOND = ;
    public static final double WRIST_ROTATION_TIMEOUT_ENCODER_TICKS_PER_SECOND = ;

    //find and add wrist
    public static final double ARM_TIMEOUT_BASE_VALUE = ;

    public static final double ARM_DEGREES_TO_ENCODER_UNITS = ;
    public static final double WRIST_DEGREES_TO_ENCODER_UNITS = ;

    public static final double ARM_ROTATION_MANUAL_DEGREES_PER_SECOND = ;
    public static final double ARM_ROTATION_MANUAL_NATIVE_UNITS_PER_TICK = ARM_ROTATION_MANUAL_DEGREES_PER_SECOND * ARM_DEGREES_TO_ENCODER_UNITS / 50;
    public static final double WRIST_ROTATION_MANUAL_DEGREES_PER_SECOND = ;
    public static final double WRIST_ROTATION_MANUAL_NATIVE_UNITS_PER_TICK = WRIST_ROTATION_MANUAL_DEGREES_PER_SECOND * WRIST_DEGREES_TO_ENCODER_UNITS / 50;


    public static final Gains armRotationGains = new Gains(0.00000000000001, 0.00000000000001, 0.00000000000001, 0.00000000000001, 1, 0.00000000000001);
    public static final Gains wristRotationGains = new Gains(0.00000000000001, 0.00000000000001, 0.00000000000001, 0.00000000000001, 1, 0.00000000000001);
    
    public static final double SENSOR_TO_MECHANISM_RATIO_ARM_MOTOR = ;
    public static final double SENSOR_TO_MECHANISM_RATIO_WRIST_MOTOR = ;


//  public static final int COUNTS_PER_REVOLUTION = 4096; ???
    public static final double COUNTS_PER_REVOLUTION = ;

}
