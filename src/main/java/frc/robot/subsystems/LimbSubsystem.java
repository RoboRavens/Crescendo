// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.util.arm.LimbPose;
import frc.robot.util.arm.ArmSetpoint;

public class LimbSubsystem extends SubsystemBase {

  private TalonFX armRotationMotor= new TalonFX(RobotMap.ARM_ROTATION_MOTOR);
  private TalonFX wristRotationMotor = new TalonFX(RobotMap.WRIST_ROTATION_MOTOR);

  private PIDController pidController;
  CommandXboxController _controller;

  private double _armRotationPosition;
  //private double _armRotationVelocity;
  //private double _armRotationAcceleration;

  private double _wristRotationPosition;

  private LimbPose limbPose = new LimbPose(Constants.ARM_STARTING_DEGREES);

  private double armRotationSubSetpointFinalTargetNativeUnits = 0;
  private double armRotationFinalTargetNativeUnits = 0;
  private double armRotationInstantaneousTargetNativeUnits = 0;

  private double wristRotationSubSetpointFinalTargetNativeUnits = 0;
  private double wristRotationFinalTargetNativeUnits = 0;
  private double wristRotationInstantaneousTargetNativeUnits = 0;

  private double rotationAFF = 0;

  public double rotationTestPower = 0;

  
  private TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
  private FeedbackConfigs feedback = new FeedbackConfigs();
  private Slot0Configs armGainsSlots = new Slot0Configs();
  private Slot1Configs wristGainsSlots = new Slot1Configs();

  private SoftwareLimitSwitchConfigs softwareLimitSwitch = new SoftwareLimitSwitchConfigs();
  private MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();


  /** Creates a new ArmSubsystem. */
  public LimbSubsystem() {

    armRotationMotor.getConfigurator().apply(talonFXConfiguration.MotionMagic);
    armRotationMotor.setInverted(true);
    //which feedback sensor
    armRotationMotor.getConfigurator().apply(feedback.withSensorToMechanismRatio(Constants.SENSOR_TO_MECHANISM_RATIO_ARM_MOTOR));
    armRotationMotor.getConfigurator().apply(armGainsSlots.withKD(Constants.armRotationGains.kD));
    armRotationMotor.getConfigurator().apply(armGainsSlots.withKG(Constants.armRotationGains.kF));
    armRotationMotor.getConfigurator().apply(armGainsSlots.withKI(Constants.armRotationGains.kI));
    armRotationMotor.getConfigurator().apply(armGainsSlots.withKP(Constants.armRotationGains.kP));
    armRotationMotor.getConfigurator().setPosition(0);

    wristRotationMotor.getConfigurator().apply(talonFXConfiguration.MotionMagic);
    wristRotationMotor.setInverted(true);
    //which feedback sensor
    wristRotationMotor.getConfigurator().apply(feedback.withSensorToMechanismRatio(Constants.SENSOR_TO_MECHANISM_RATIO_WRIST_MOTOR));
    wristRotationMotor.getConfigurator().apply(armGainsSlots.withKD(Constants.wristRotationGains.kD));
    wristRotationMotor.getConfigurator().apply(armGainsSlots.withKG(Constants.wristRotationGains.kF));
    wristRotationMotor.getConfigurator().apply(armGainsSlots.withKI(Constants.wristRotationGains.kI));
    wristRotationMotor.getConfigurator().apply(armGainsSlots.withKP(Constants.wristRotationGains.kP));
    wristRotationMotor.getConfigurator().apply(wristGainsSlots);
    wristRotationMotor.getConfigurator().setPosition(0);

   // armRotationMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
   // wristRotationMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
    armRotationMotor.getConfigurator().apply(softwareLimitSwitch.withForwardSoftLimitEnable(true));
    armRotationMotor.getConfigurator().apply(softwareLimitSwitch.withReverseSoftLimitEnable(true));
    armRotationMotor.getConfigurator().apply(softwareLimitSwitch.withReverseSoftLimitThreshold(Constants.ARM_ROTATION_MAXIMUM_ENCODER_UNITS));
    armRotationMotor.getConfigurator().apply(softwareLimitSwitch.withForwardSoftLimitThreshold(Constants.ARM_ROTATION_MAXIMUM_ENCODER_UNITS));

    wristRotationMotor.getConfigurator().apply(softwareLimitSwitch.withForwardSoftLimitEnable(true));
    wristRotationMotor.getConfigurator().apply(softwareLimitSwitch.withReverseSoftLimitEnable(true));
    wristRotationMotor.getConfigurator().apply(softwareLimitSwitch.withReverseSoftLimitThreshold(Constants.WRIST_ROTATION_MAXIMUM_ENCODER_UNITS));
    wristRotationMotor.getConfigurator().apply(softwareLimitSwitch.withForwardSoftLimitThreshold(Constants.WRIST_ROTATION_MAXIMUM_ENCODER_UNITS));
  }

  public void enableArmRotationLimit(boolean ignoreRotationLimit) {
    armRotationMotor.getConfigurator().apply(softwareLimitSwitch.withForwardSoftLimitEnable(ignoreRotationLimit));
    armRotationMotor.getConfigurator().apply(softwareLimitSwitch.withReverseSoftLimitEnable(ignoreRotationLimit));
  }
  public void enableWristRotationLimit(boolean ignoreRotationLimit) {
    wristRotationMotor.getConfigurator().apply(softwareLimitSwitch.withForwardSoftLimitEnable(ignoreRotationLimit));
    wristRotationMotor.getConfigurator().apply(softwareLimitSwitch.withReverseSoftLimitEnable(ignoreRotationLimit));
  }

  public void setFinalArmRotationSetpoint(double setpoint) {
    armRotationFinalTargetNativeUnits = setpoint;
  }
    public void setFinalWristRotationSetpoint(double setpoint) {
    wristRotationFinalTargetNativeUnits = setpoint;
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
        armRotationFinalTargetNativeUnits = Math.min(armRotationFinalTargetNativeUnits, limbPose.getArmRotationMaximumBoundNativeUnits());
    }
    else {
        // The arm is moving backward.
        armRotationFinalTargetNativeUnits = Math.max(armRotationFinalTargetNativeUnits, limbPose.getArmRotationMinimumBoundNativeUnits());
    }
//change for wrist
    if (_armRotationPosition < armRotationFinalTargetNativeUnits) {
        // The wrist is moving forward.
        armRotationFinalTargetNativeUnits = Math.min(armRotationFinalTargetNativeUnits, limbPose.getArmRotationMaximumBoundNativeUnits());
    }
    else {
        // The wrist is moving backward.
        armRotationFinalTargetNativeUnits = Math.max(armRotationFinalTargetNativeUnits, limbPose.getArmRotationMinimumBoundNativeUnits());
    }
}

  public void updateWristFinalTargetPositions() {
    // Rotation is slightly trickier since it has constraints in either direction.
    // First, figure out which way the arm is rotating by checking the difference between the target and the actual.
    // Then, find the constraint that is in the proper direction relative to the current position out of the two constraints.
    // Remember to use max instead of min when looking at bounds that are lower than the current value.
    if (_wristRotationPosition < wristRotationFinalTargetNativeUnits) {
        // The arm is moving forward.
        wristRotationFinalTargetNativeUnits = Math.min(wristRotationFinalTargetNativeUnits, limbPose.getWristRotationMaximumBoundNativeUnits());
    }
    else {
        // The arm is moving backward.
        wristRotationFinalTargetNativeUnits = Math.max(wristRotationFinalTargetNativeUnits, limbPose.getWristRotationMinimumBoundNativeUnits());
    }
//change for wrist
    if (_wristRotationPosition < wristRotationFinalTargetNativeUnits) {
        // The wrist is moving forward.
        wristRotationFinalTargetNativeUnits = Math.min(wristRotationFinalTargetNativeUnits, limbPose.getWristRotationMaximumBoundNativeUnits());
    }
    else {
        // The wrist is moving backward.
        wristRotationFinalTargetNativeUnits = Math.max(wristRotationFinalTargetNativeUnits, limbPose.getWristRotationMinimumBoundNativeUnits());
    }
}

public void setArmRotationPosition(double setpoint, double armRotationVelocity, double armRotationAcceleration) {
  armRotationMotor.getConfigurator().apply(motionMagicConfigs.withMotionMagicCruiseVelocity(armRotationVelocity));
  armRotationMotor.getConfigurator().apply(motionMagicConfigs.withMotionMagicAcceleration(armRotationAcceleration));
  armRotationMotor.set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, rotationAFF);
} 
public void setWristRotationPosition(double setpoint, double wristRotationVelocity, double wristRotationAcceleration) {
  wristRotationMotor.getConfigurator().apply(motionMagicConfigs.withMotionMagicCruiseVelocity(wristRotationVelocity));
  wristRotationMotor.getConfigurator().apply(motionMagicConfigs.withMotionMagicAcceleration(wristRotationAcceleration));
  wristRotationMotor.set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, rotationAFF);
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
  double armRotationDifference = Math.abs(armRotationMotor.getPosition().getValueAsDouble() - armRotationFinalTargetNativeUnits);
  double wristRotationDifference = Math.abs(wristRotationMotor.getPosition().getValueAsDouble() - wristRotationFinalTargetNativeUnits);


  double wristRotationTime = wristRotationDifference / Constants.WRIST_ROTATION_TIMEOUT_ENCODER_TICKS_PER_SECOND;
  double armRotationTime = armRotationDifference / Constants.ARM_ROTATION_TIMEOUT_ENCODER_TICKS_PER_SECOND;
  
  return Math.max(wristRotationTime, armRotationTime) + Constants.ARM_TIMEOUT_BASE_VALUE;
}



  @Override
  public void periodic() {

    limbPose.calculateInstantaneousMaximums();
    this.updateInstantaneousMaximums();
    // this.updateFinalTargetPositions();
    this.updateAFFs();
    // This method will be called once per scheduler run
  }

  private void updateAFFs() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateAFFs'");
  }

  private void updateInstantaneousMaximums() {

    // Update based on which direction the arm is moving.
    //arm
    if (limbPose.getArmRotationNativeUnits() < this.armRotationFinalTargetNativeUnits) {
        armRotationInstantaneousTargetNativeUnits = Math.min(limbPose.getArmRotationMaximumBoundNativeUnits(), armRotationFinalTargetNativeUnits);
    }
    else {
        armRotationInstantaneousTargetNativeUnits = Math.max(limbPose.getArmRotationMinimumBoundNativeUnits(), armRotationFinalTargetNativeUnits);
    }

    //wrist
    if (limbPose.getWristRotationNativeUnits() < this.wristRotationFinalTargetNativeUnits) {
        wristRotationFinalTargetNativeUnits = Math.min(limbPose.getWristRotationMaximumBoundNativeUnits(), wristRotationFinalTargetNativeUnits);
    }
    else {
        wristRotationFinalTargetNativeUnits = Math.max(limbPose.getWristRotationMaximumBoundNativeUnits(), wristRotationFinalTargetNativeUnits);
    }

  }

  //add wrist??
    public double getArmCurrentAngleDegrees() {
    return armRotationMotor.getPosition().getValueAsDouble() / Constants.ARM_DEGREES_TO_ENCODER_UNITS;
  }
  public double getArmCurrentRotationNativeUnits() {
    return armRotationMotor.getPosition().getValueAsDouble();
  }

      public double getWristCurrentAngleDegrees() {
    return wristRotationMotor.getPosition().getValueAsDouble() / Constants.ARM_DEGREES_TO_ENCODER_UNITS;
  }
  public double getWristCurrentRotationNativeUnits() {
    return wristRotationMotor.getPosition().getValueAsDouble();
  }

  public void increaseArmRotationTargetManually() {
        armRotationFinalTargetNativeUnits -= Constants.ARM_ROTATION_MANUAL_NATIVE_UNITS_PER_TICK;
  }
  public void decreaseArmRotationTargetManually() {
        armRotationFinalTargetNativeUnits += Constants.ARM_ROTATION_MANUAL_NATIVE_UNITS_PER_TICK;
  }
  public void increaseWristRotationTargetManually() {
        armRotationFinalTargetNativeUnits -= Constants.WRIST_ROTATION_MANUAL_NATIVE_UNITS_PER_TICK;
  }
  public void decreaseWristRotationTargetManually() {
        armRotationFinalTargetNativeUnits += Constants.WRIST_ROTATION_MANUAL_NATIVE_UNITS_PER_TICK;
  }

  public double getPositionFromAngle(double intendedAngle) {
    double position = (intendedAngle/360) * Constants.COUNTS_PER_REVOLUTION;
      return position;
  }

  public double getArmRotationFinalTargetNativeUnits() {
    return this.armRotationFinalTargetNativeUnits;
  }
  public double getArmRotationInstantaneousTargetNativeUnits() {
    return armRotationInstantaneousTargetNativeUnits;
  }
  public void setArmRotationVoltage(double voltage) {
    armRotationMotor.setVoltage(voltage);
  }

  public double getWristRotationFinalTargetNativeUnits() {
    return this.armRotationFinalTargetNativeUnits;
  }

  public double getWristRotationInstantaneousTargetNativeUnits() {
    return armRotationInstantaneousTargetNativeUnits;
  }
    public void setWristRotationVoltage(double voltage) {
    armRotationMotor.setVoltage(voltage);
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
