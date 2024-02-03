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
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import frc.robot.util.arm.LimbSetpoint;

public class WristSubsystem extends SubsystemBase {

  private TalonFX wristRotationMotor = new TalonFX(RobotMap.WRIST_ROTATION_MOTOR);

  private PIDController pidController;
  CommandXboxController _controller;

  private double _wristRotationPosition;

  private LimbPose limbPose = new LimbPose(Constants.WRIST_STARTING_DEGREES);

  private double wristRotationSubSetpointFinalTargetNativeUnits = 0;
  private double wristRotationFinalTargetNativeUnits = 0;
  private double wristRotationInstantaneousTargetNativeUnits = 0;

  private double rotationAFF = 0;

  public double rotationTestPower = 0;

  
  private TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
  private FeedbackConfigs feedback = new FeedbackConfigs();
  private Slot0Configs Slot0Configs = new Slot0Configs();

  private SoftwareLimitSwitchConfigs softwareLimitSwitch = new SoftwareLimitSwitchConfigs();
  private MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);


  /** Creates a new ArmSubsystem. */
  public WristSubsystem() {

    wristRotationMotor.getConfigurator().apply(talonFXConfiguration.MotionMagic);
    wristRotationMotor.setInverted(true);
    wristRotationMotor.getConfigurator().apply(Slot0Configs
          .withKD(Constants.wristRotationGains.kD)
          .withKI(Constants.wristRotationGains.kI)
          .withKP(Constants.wristRotationGains.kP)
          .withKG(Constants.wristRotationGains.kF));
    wristRotationMotor.getConfigurator().setPosition(0);

   // armRotationMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
   // wristRotationMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);

    wristRotationMotor.getConfigurator().apply(softwareLimitSwitch.withForwardSoftLimitEnable(true));
    wristRotationMotor.getConfigurator().apply(softwareLimitSwitch.withReverseSoftLimitEnable(true));
    wristRotationMotor.getConfigurator().apply(softwareLimitSwitch.withReverseSoftLimitThreshold(Constants.WRIST_ROTATION_MINIMUM_ENCODER_UNITS));
    wristRotationMotor.getConfigurator().apply(softwareLimitSwitch.withForwardSoftLimitThreshold(Constants.WRIST_ROTATION_MAXIMUM_ENCODER_UNITS));
  }

  public void enableWristRotationLimit(boolean ignoreRotationLimit) {
    wristRotationMotor.getConfigurator().apply(softwareLimitSwitch.withForwardSoftLimitEnable(ignoreRotationLimit));
    wristRotationMotor.getConfigurator().apply(softwareLimitSwitch.withReverseSoftLimitEnable(ignoreRotationLimit));
  }
    public void setFinalWristRotationSetpoint(double setpoint) {
    wristRotationFinalTargetNativeUnits = setpoint;
  }
  
  public void setFinalTargetPositions(LimbSetpoint finalSetpoint, LimbSetpoint currentSetpoint) {
    wristRotationFinalTargetNativeUnits = finalSetpoint.getRotationSetpoint();
    wristRotationSubSetpointFinalTargetNativeUnits = currentSetpoint.getRotationSetpoint();
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
public void setWristRotationPosition(double setpoint, double wristRotationVelocity, double wristRotationAcceleration) {
  wristRotationMotor.getConfigurator().apply(motionMagicConfigs
        .withMotionMagicCruiseVelocity(wristRotationVelocity)
        .withMotionMagicAcceleration(wristRotationAcceleration));
  wristRotationMotor.setControl(motionMagicVoltage.withPosition(setpoint));
}

//
public void stopWristRotation() {
  wristRotationMotor.stopMotor();
}
//
public void armFullStop() {
  stopWristRotation();
}
//?
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
  double wristRotationDifference = Math.abs(wristRotationMotor.getPosition().getValueAsDouble() - wristRotationFinalTargetNativeUnits);
  double wristRotationTime = wristRotationDifference / Constants.WRIST_ROTATION_TIMEOUT_ENCODER_TICKS_PER_SECOND;
  return wristRotationTime + Constants.WRIST_TIMEOUT_BASE_VALUE;
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
    //wrist
    if (limbPose.getWristRotationNativeUnits() < this.wristRotationFinalTargetNativeUnits) {
        wristRotationFinalTargetNativeUnits = Math.min(limbPose.getWristRotationMaximumBoundNativeUnits(), wristRotationFinalTargetNativeUnits);
    }
    else {
        wristRotationFinalTargetNativeUnits = Math.max(limbPose.getWristRotationMaximumBoundNativeUnits(), wristRotationFinalTargetNativeUnits);
    }

  }
      public double getWristCurrentAngleDegrees() {
    return wristRotationMotor.getPosition().getValueAsDouble() / Constants.ELBOW_DEGREES_TO_ENCODER_UNITS;
  }
  public double getWristCurrentRotationNativeUnits() {
    return wristRotationMotor.getPosition().getValueAsDouble();
  }
  public double getPositionFromAngle(double intendedAngle) {
    double position = (intendedAngle/360) * Constants.COUNTS_PER_REVOLUTION;
      return position;
  }
  public double getWristRotationFinalTargetNativeUnits() {
    return this.wristRotationFinalTargetNativeUnits;
  }
  public void setWristRotationVoltage(double voltage) {
    wristRotationMotor.setVoltage(voltage);
  }

public void setRotationVoltage(double voltage) {
    wristRotationMotor.setVoltage(voltage);
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
