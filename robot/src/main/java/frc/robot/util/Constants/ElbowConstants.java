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

  public static final double DEGREES_GROUND_PICKUP = 0.0;
  public static final double DEGREES_SPEAKER_SCORING = 0.0; //?
  public static final double DEGREES_SPEAKER_SCORING_ARM_UP = 0.0; //?
  public static final double DEGREES_AMP_SCORE = 95;
  public static final double DEGREES_TRAP_LOAD_FROM_SOURCE = 50;
  public static final double DEGREES_SOURCE_LOAD = 50;
  public static final double DEGREES_TRAP_SCORE = 90;
  public static final double DEREES_START_CONFIG = 68;

  public static final double MOTOR_POWER_FEEDFORWARD_AT_HORIZONTAL = .025;
  public static final double MOTOR_POWER_DIRECTION_TO_GO_UP_FROM_HORIZONTAL = -1;

  public static Slot0Configs getSlot0Configs() {
    var slot0Config = new Slot0Configs();
    slot0Config.kP = Constants.ELBOW_PID.kP;
    slot0Config.kI = Constants.ELBOW_PID.kI;
    slot0Config.kD = Constants.ELBOW_PID.kD;
    return slot0Config;
  }


}
