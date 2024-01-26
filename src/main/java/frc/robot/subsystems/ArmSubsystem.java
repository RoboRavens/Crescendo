// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.util.arm.ArmPose;
import frc.robot.util.arm.ArmSetpoint;

public class ArmSubsystem extends SubsystemBase {
  

  private TalonFX armRotationMotor= new TalonFX(RobotMap.ARM_ROTATION_MOTOR);
  private TalonFX wristRotationMotor = new TalonFX(RobotMap.WRIST_ROTATION_MOTOR);

  private PIDController pidController;
  CommandXboxController _controller;

  private double _armRotationPosition;
  private double _armRotationVelocity;
  private double _armRotationAcceleration;

  private ArmPose armPose = new ArmPose(Constants.ARM_STARTING_DEGREES);
    private ArmPose wristPose = new ArmPose(Constants.WRIST_STARTING_DEGREES);

  private double armRotationSubSetpointFinalTargetNativeUnits = 0;
  private double armRotationFinalTargetNativeUnits = 0;
  private double armRotationInstantaneousTargetNativeUnits = 0;

  private double wristRotationSubSetpointFinalTargetNativeUnits = 0;
  private double wristRotationFinalTargetNativeUnits = 0;
  private double wristRotationInstantaneousTargetNativeUnits = 0;

  private double rotationAFF = 0;

  public double rotationTestPower = 0;


  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {


   configFactoryDefault(armRotationMotor);
   configFactoryDefault(wristRotationMotor);

    armRotationMotor.setInverted(true);
    wristRotationMotor.setInverted(true);


    armRotationMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    armRotationMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
    armRotationMotor.config_kF(Constants.kSlotIdx, Constants.armRotationGains.kF, Constants.kTimeoutMs);
    armRotationMotor.config_kP(Constants.kSlotIdx, Constants.armRotationGains.kP, Constants.kTimeoutMs);
    armRotationMotor.config_kI(Constants.kSlotIdx, Constants.armRotationGains.kI, Constants.kTimeoutMs);
    armRotationMotor.config_kD(Constants.kSlotIdx, Constants.armRotationGains.kD, Constants.kTimeoutMs);
    wristRotationMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    wristRotationMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
    wristRotationMotor.config_kF(Constants.kSlotIdx, Constants.wristRotationGains.kF, Constants.kTimeoutMs);
    wristRotationMotor.config_kP(Constants.kSlotIdx, Constants.wristRotationGains.kP, Constants.kTimeoutMs);
    wristRotationMotor.config_kI(Constants.kSlotIdx, Constants.wristRotationGains.kI, Constants.kTimeoutMs);
    wristRotationMotor.config_kD(Constants.kSlotIdx, Constants.wristRotationGains.kD, Constants.kTimeoutMs);
    wristRotationMotor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    armRotationMotor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    wristRotationMotor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    armRotationMotor.configForwardSoftLimitThreshold(Constants.ARM_ROTATION_MAXIMUM_ENCODER_UNITS, 0);
    armRotationMotor.configReverseSoftLimitThreshold(Constants.ARM_ROTATION_MAXIMUM_ENCODER_UNITS * -1, 0);
    armRotationMotor.configForwardSoftLimitEnable(true, 0);
    armRotationMotor.configReverseSoftLimitEnable(true, 0);

    wristRotationMotor.configForwardSoftLimitThreshold(Constants.WRIST_ROTATION_MAXIMUM_ENCODER_UNITS, 0);
    wristRotationMotor.configReverseSoftLimitThreshold(Constants.WRIST_ROTATION_MAXIMUM_ENCODER_UNITS * -1, 0);
    wristRotationMotor.configForwardSoftLimitEnable(true, 0);
    wristRotationMotor.configReverseSoftLimitEnable(true, 0);
  }

  public void enableArmRotationLimit(boolean ignoreRotationLimit) {
    armRotationMotor.configForwardSoftLimitEnable(ignoreRotationLimit);
    armRotationMotor.configReverseSoftLimitEnable(ignoreRotationLimit);
  }
  public void enableWristRotationLimit(boolean ignoreRotationLimit) {
    wristRotationMotor.configForwardSoftLimitEnable(ignoreRotationLimit);
    wristRotationMotor.configReverseSoftLimitEnable(ignoreRotationLimit);
  }

  public void setFinalArmRotationSetpoint(double setpoint) {
    armRotationFinalTargetNativeUnits = setpoint;
  }

  public void setFinalTargetPositions(ArmSetpoint finalSetpoint, ArmSetpoint currentSetpoint) {
    armRotationFinalTargetNativeUnits = finalSetpoint.getArmRotationSetpoint();
    wristRotationFinalTargetNativeUnits = finalSetpoint.getWristRotationSetpoint();
    armRotationSubSetpointFinalTargetNativeUnits = currentSetpoint.getArmRotationSetpoint();
    wristRotationSubSetpointFinalTargetNativeUnits = currentSetpoint.getWristRotationSetpoint();
  }

  public void updateArmFinalTargetPositions() {
    // Rotation is slightly trickier since it has constraints in either direction.
    // First, figure out which way the arm is rotating by checking the difference between the target and the actual.
    // Then, find the constraint that is in the proper direction relative to the current position out of the two constraints.
    // Remember to use max instead of min when looking at bounds that are lower than the current value.
    if (_armRotationPosition < armRotationFinalTargetNativeUnits) {
        // The arm is moving forward.
        armRotationFinalTargetNativeUnits = Math.min(armRotationFinalTargetNativeUnits, armPose.getArmRotationMaximumBoundNativeUnits());
    }
    else {
        // The arm is moving backward.
        armRotationFinalTargetNativeUnits = Math.max(armRotationFinalTargetNativeUnits, armPose.getArmRotationMinimumBoundNativeUnits());
    }
//change for wrist
    if (_armRotationPosition < armRotationFinalTargetNativeUnits) {
        // The wrist is moving forward.
        armRotationFinalTargetNativeUnits = Math.min(armRotationFinalTargetNativeUnits, armPose.getArmRotationMaximumBoundNativeUnits());
    }
    else {
        // The wrist is moving backward.
        armRotationFinalTargetNativeUnits = Math.max(armRotationFinalTargetNativeUnits, armPose.getArmRotationMinimumBoundNativeUnits());
    }
}

  public void updateWristFinalTargetPositions() {
    // Rotation is slightly trickier since it has constraints in either direction.
    // First, figure out which way the arm is rotating by checking the difference between the target and the actual.
    // Then, find the constraint that is in the proper direction relative to the current position out of the two constraints.
    // Remember to use max instead of min when looking at bounds that are lower than the current value.
    if (_armRotationPosition < armRotationFinalTargetNativeUnits) {
        // The arm is moving forward.
        armRotationFinalTargetNativeUnits = Math.min(armRotationFinalTargetNativeUnits, armPose.getWristRotationMaximumBoundNativeUnits());
    }
    else {
        // The arm is moving backward.
        armRotationFinalTargetNativeUnits = Math.max(armRotationFinalTargetNativeUnits, armPose.getWristRotationMinimumBoundNativeUnits());
    }
//change for wrist
    if (_armRotationPosition < armRotationFinalTargetNativeUnits) {
        // The wrist is moving forward.
        armRotationFinalTargetNativeUnits = Math.min(armRotationFinalTargetNativeUnits, armPose.getWristRotationMaximumBoundNativeUnits());
    }
    else {
        // The wrist is moving backward.
        armRotationFinalTargetNativeUnits = Math.max(armRotationFinalTargetNativeUnits, armPose.getWristRotationMinimumBoundNativeUnits());
    }
}

public void setArmRotationPosition(double setpoint, double rotationVelocity, double rotationAcceleration) {
  wristRotationMotor.configMotionCruiseVelocity(rotationVelocity, Constants.kTimeoutMs);
  wristRotationMotor.configMotionAcceleration(rotationAcceleration, Constants.kTimeoutMs);
  wristRotationMotor.set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, rotationAFF);

}
public void setWristRotationPosition(double setpoint, double rotationVelocity, double rotationAcceleration) {
  armRotationMotor.configMotionCruiseVelocity(rotationVelocity, Constants.kTimeoutMs);
  armRotationMotor.configMotionAcceleration(rotationAcceleration, Constants.kTimeoutMs);
  armRotationMotor.set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, rotationAFF);

}

//
public void stopWristRotation() {
  wristRotationMotor.stopMotor();
}
public void stopArmRotation() {
  armRotationMotor.stopMotor();
}

//
public void armFullStop() {
  stopArmRotation();
  stopWristRotation();
}

public void armMotionMagic() {
  pidController = new PIDController(Constants.armRotationGains.kP, Constants.armRotationGains.kI, Constants.armRotationGains.kD);
  pidController.setSetpoint(8);
  pidController.setP(0.0);
  pidController.getPositionError();
  pidController.getPositionTolerance();
  pidController.getSetpoint();
  pidController.getP();
  pidController.getI();
  pidController.getD();
}

//
public void wristMotionMagic() {
  pidController = new PIDController(Constants.wristRotationGains.kP, Constants.wristRotationGains.kI, Constants.wristRotationGains.kD);
  pidController.setSetpoint(8);
  pidController.setP(0.0);
  pidController.getPositionError();
  pidController.getPositionTolerance();
  pidController.getSetpoint();
  pidController.getP();
  pidController.getI();
  pidController.getD();
}

public double getCommandTimeoutSeconds() {
  double armRotationDifference = Math.abs(armRotationMotor.getSelectedSensorPosition() - armRotationFinalTargetNativeUnits);
  double wristRotationDifference = Math.abs(wristRotationMotor.getSelectedSensorPosition() - wristRotationFinalTargetNativeUnits);


  double wristRotationTime = wristRotationDifference / Constants.WRIST_ROTATION_TIMEOUT_ENCODER_TICKS_PER_SECOND;
  double armRotationTime = armRotationDifference / Constants.ARM_ROTATION_TIMEOUT_ENCODER_TICKS_PER_SECOND;
  
  return Math.max(wristRotationTime, armRotationTime) + Constants.ARM_TIMEOUT_BASE_VALUE;
}



  @Override
  public void periodic() {

    armPose.calculateInstantaneousMaximums();
    this.updateInstantaneousMaximums();
    // this.updateFinalTargetPositions();
    this.updateAFFs();
    // This method will be called once per scheduler run
  }

  private void updateInstantaneousMaximums() {

    // Update based on which direction the arm is moving.
    //arm
    if (armPose.getArmRotationNativeUnits() < this.armRotationFinalTargetNativeUnits) {
        armRotationInstantaneousTargetNativeUnits = Math.min(armPose.getArmRotationMaximumBoundNativeUnits(), armRotationFinalTargetNativeUnits);
    }
    else {
        armRotationInstantaneousTargetNativeUnits = Math.max(armPose.getArmRotationMinimumBoundNativeUnits(), armRotationFinalTargetNativeUnits);
    }

    //wrist
    if (armPose.getArmRotationNativeUnits() < this.armRotationFinalTargetNativeUnits) {
        wristRotationFinalTargetNativeUnits = Math.min(armPose.getArmRotationMaximumBoundNativeUnits(), wristRotationFinalTargetNativeUnits);
    }
    else {
        wristRotationFinalTargetNativeUnits = Math.max(armPose.getArmRotationMinimumBoundNativeUnits(), wristRotationFinalTargetNativeUnits);
    }

  }

  //  
  public void configFactoryDefault(TalonFX motor) {
    TalonFXConfigurator configurator = motor.getConfigurator();

    talonFxConfigs = new TalonFXConfiguration();
    recordResponseCode("resetTalonFxConfigs", configurator.apply(talonFxConfigs));
  }
    
  public void configsFeedback(TalonFX motor) {
    TalonFXConfigurator configurator = motor.getConfigurator();

    feedbackConfigs = new FeedbackConfigs();
    recordResponseCode("resetFeedbackConfigs", configurator.apply(feedbackConfigs));
  }
  



  /*
  public void updateAFFs() {
    updateRotationAFF();
  }
   */

  /*
  public void updateRotationAFF() {
    double armExtensionPercentTerm = (1 - Constants.ARM_MINIMUM_EXTENSION_RATIO) * extensionPercent;
    double extensionScaling = Constants.ARM_MINIMUM_EXTENSION_RATIO + armExtensionPercentTerm;

    double rotationScaling = Math.sin(Math.abs(armPose.getArmAngleRadians()));

    double maxAFF = Constants.ROTATION_SIDEWAYS_EMPTY_AFF;

    if (this.getCurrentAngleDegrees() > 0) {
        maxAFF = maxAFF * -1;
    }

    rotationAFF = maxAFF * rotationScaling * extensionScaling;
  }
   */
  

}
