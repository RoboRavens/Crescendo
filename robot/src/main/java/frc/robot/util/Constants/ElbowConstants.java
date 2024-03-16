package frc.robot.util.Constants;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.ctre.phoenix6.configs.Slot0Configs;

public class ElbowConstants {

  public static final double ENCODER_POSITION_AT_GROUND_PICKUP = 0;
  public static final double ENCODER_POSITION_AT_HORIZONTAL = -5.27;
  public static final double ENCODER_POSITION_AT_VERTICAL = -35.51;
  public static final double ENCODER_LIMIT_RESET_BUFFER = .5;
  public static final double ENCODER_POSITION_AT_STARTUP = -22.496582;
  
  // DEGREES_FLOOR_PICKUP in onshape is 0, but -15.684524 in below table
  // all other values from onshape table have been reduced by 15 degrees
  // these constants show degrees from horizontal
  // public static final double DEGREES_FLOOR_PICKUP = -15.684524;
  public static final double DEGREES_FLOOR_PICKUP = -13.684524;
  public static final double DEGREES_AMP_SCORE = 80;
  public static final double DEGREES_TRAP_LOAD_FROM_SOURCE = 35;
  public static final double DEGREES_SOURCE_LOAD = 35;
  public static final double DEGREES_TRAP_SCORE = 75;
  public static final double DEGREES_START_CONFIG = 53;
  public static final double DEGREES_SOUTH_CENTER_PRELOAD = 58;
  public static final double DEGREES_START_CONFIG_UP = 85;

  public static final double RADIANS_FLOOR_PICKUP = Math.toRadians(DEGREES_FLOOR_PICKUP);
  public static final double RADIANS_AMP_SCORE = Math.toRadians(DEGREES_AMP_SCORE);
  public static final double RADIANS_TRAP_LOAD_FROM_SOURCE = Math.toRadians(DEGREES_TRAP_LOAD_FROM_SOURCE);
  public static final double RADIANS_SOURCE_LOAD = Math.toRadians(DEGREES_SOURCE_LOAD);
  public static final double RADIANS_TRAP_SCORE = Math.toRadians(DEGREES_TRAP_SCORE);
  public static final double RADIANS_START_CONFIG = Math.toRadians(DEGREES_START_CONFIG);
  public static final double IS_AT_SETPOINT_BUFFER = 0.5;

  public static final double JOYSTICK_CONTROL_SCALING_FACTOR = 0.15;


  public static double MOTOR_POWER_FEEDFORWARD_AT_HORIZONTAL = .025;
  public static double MOTOR_POWER_DIRECTION_TO_GO_UP_FROM_HORIZONTAL = -1;

  public static Slot0Configs getSlot0Configs() {
    var slot0Config = new Slot0Configs();
    slot0Config.kP = 1;
    slot0Config.kI = 0;
    slot0Config.kD = 0;
    return slot0Config;
  }


}
