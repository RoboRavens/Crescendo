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

public class ElbowSubsystem extends SubsystemBase {

  private TalonFX elbowRotationMotor= new TalonFX(RobotMap.ELBOW_ROTATION_MOTOR);


  private PIDController pidController;
  CommandXboxController _controller;

  private double _elbowRotationPosition;
  //private double _armRotationVelocity;
  //private double _armRotationAcceleration;

  private LimbPose limbPose = new LimbPose(Constants.ELBOW_STARTING_DEGREES);

  private double elbowRotationSubSetpointFinalTargetNativeUnits = 0;
  private double elbowRotationFinalTargetNativeUnits = 0;
  private double elbowRotationInstantaneousTargetNativeUnits = 0;

  private double rotationAFF = 0;

  public double rotationTestPower = 0;

  
  private TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
  private FeedbackConfigs feedback = new FeedbackConfigs();
  private Slot0Configs Slot0Configs = new Slot0Configs();

  private SoftwareLimitSwitchConfigs softwareLimitSwitch = new SoftwareLimitSwitchConfigs();
  private MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);


  /** Creates a new ArmSubsystem. */
  public ElbowSubsystem() {

    elbowRotationMotor.getConfigurator().apply(talonFXConfiguration.MotionMagic);
    elbowRotationMotor.setInverted(true);
    elbowRotationMotor.getConfigurator().apply(Slot0Configs
          .withKD(Constants.elbowRotationGains.kD)
          .withKI(Constants.elbowRotationGains.kI)
          .withKP(Constants.elbowRotationGains.kP)
          .withKG(Constants.elbowRotationGains.kF));
    elbowRotationMotor.getConfigurator().setPosition(0);

   // armRotationMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
   // wristRotationMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
    elbowRotationMotor.getConfigurator().apply(softwareLimitSwitch.withForwardSoftLimitEnable(true));
    elbowRotationMotor.getConfigurator().apply(softwareLimitSwitch.withReverseSoftLimitEnable(true));
    elbowRotationMotor.getConfigurator().apply(softwareLimitSwitch.withReverseSoftLimitThreshold(Constants.ELBOW_ROTATION_MINIMUM_ENCODER_UNITS));
    elbowRotationMotor.getConfigurator().apply(softwareLimitSwitch.withForwardSoftLimitThreshold(Constants.ELBOW_ROTATION_MAXIMUM_ENCODER_UNITS));
  }

  public void enableArmRotationLimit(boolean ignoreRotationLimit) {
    elbowRotationMotor.getConfigurator().apply(softwareLimitSwitch.withForwardSoftLimitEnable(ignoreRotationLimit));
    elbowRotationMotor.getConfigurator().apply(softwareLimitSwitch.withReverseSoftLimitEnable(ignoreRotationLimit));
  }

  public void setFinalArmRotationSetpoint(double setpoint) {
    elbowRotationFinalTargetNativeUnits = setpoint;
  }


  public void setFinalTargetPositions(LimbSetpoint finalSetpoint, LimbSetpoint currentSetpoint) {
    elbowRotationFinalTargetNativeUnits = finalSetpoint.getRotationSetpoint();
    elbowRotationSubSetpointFinalTargetNativeUnits = currentSetpoint.getRotationSetpoint();
  }

  public void updateArmFinalTargetPositions() {
    // Rotation is slightly trickier since it has constraints in either direction.
    // First, figure out which way the arm is rotating by checking the difference between the target and the actual.
    // Then, find the constraint that is in the proper direction relative to the current position out of the two constraints.
    // Remember to use max instead of min when looking at bounds that are lower than the current value.
    if (_elbowRotationPosition < elbowRotationFinalTargetNativeUnits) {
        // The arm is moving forward.
        elbowRotationFinalTargetNativeUnits = Math.min(elbowRotationFinalTargetNativeUnits, limbPose.getElbowRotationMaximumBoundNativeUnits());
    }
    else {
        // The arm is moving backward.
        elbowRotationFinalTargetNativeUnits = Math.max(elbowRotationFinalTargetNativeUnits, limbPose.getElbowRotationMinimumBoundNativeUnits());
    }
//change for wrist
    if (_elbowRotationPosition < elbowRotationFinalTargetNativeUnits) {
        // The wrist is moving forward.
        elbowRotationFinalTargetNativeUnits = Math.min(elbowRotationFinalTargetNativeUnits, limbPose.getElbowRotationMaximumBoundNativeUnits());
    }
    else {
        // The wrist is moving backward.
        elbowRotationFinalTargetNativeUnits = Math.max(elbowRotationFinalTargetNativeUnits, limbPose.getElbowRotationMinimumBoundNativeUnits());
    }
}

public void setElbowRotationPosition(double setpoint, double elbowRotationVelocity, double elbowRotationAcceleration) {
  elbowRotationMotor.getConfigurator().apply(motionMagicConfigs
        .withMotionMagicCruiseVelocity(elbowRotationVelocity)
        .withMotionMagicAcceleration(elbowRotationAcceleration));
  elbowRotationMotor.setControl(motionMagicVoltage.withPosition(setpoint));
} 


public void stopElbowRotation() {
  elbowRotationMotor.stopMotor();
}

//
public void armFullStop() {
  stopElbowRotation();
}

//?
public void elbowMotionMagic() {
  pidController = new PIDController(Constants.elbowRotationGains.kP, Constants.elbowRotationGains.kI, Constants.elbowRotationGains.kD);
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
  double elbowRotationDifference = Math.abs(elbowRotationMotor.getPosition().getValueAsDouble() - elbowRotationFinalTargetNativeUnits);
  double elbowRotationTime = elbowRotationDifference / Constants.ELBOW_ROTATION_TIMEOUT_ENCODER_TICKS_PER_SECOND;
  
  return elbowRotationTime + Constants.ELBOW_TIMEOUT_BASE_VALUE;
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
    if (limbPose.getElbowRotationNativeUnits() < this.elbowRotationFinalTargetNativeUnits) {
        elbowRotationInstantaneousTargetNativeUnits = Math.min(limbPose.getElbowRotationMaximumBoundNativeUnits(), elbowRotationFinalTargetNativeUnits);
    }
    else {
        elbowRotationInstantaneousTargetNativeUnits = Math.max(limbPose.getElbowRotationMinimumBoundNativeUnits(), elbowRotationFinalTargetNativeUnits);
    }
  }

  //add wrist??
    public double getElbowCurrentAngleDegrees() {
    return elbowRotationMotor.getPosition().getValueAsDouble() / Constants.ELBOW_DEGREES_TO_ENCODER_UNITS;
  }
  public double getElbowCurrentRotationNativeUnits() {
    return elbowRotationMotor.getPosition().getValueAsDouble();
  }
  public void increaseElbowRotationTargetManually() {
        elbowRotationFinalTargetNativeUnits -= Constants.ELBOW_ROTATION_MANUAL_NATIVE_UNITS_PER_TICK;
  }
  public void decreaseElbowRotationTargetManually() {
        elbowRotationFinalTargetNativeUnits += Constants.ELBOW_ROTATION_MANUAL_NATIVE_UNITS_PER_TICK;
  }
  public double getPositionFromAngle(double intendedAngle) {
    double position = (intendedAngle/360) * Constants.COUNTS_PER_REVOLUTION;
      return position;
  }

  public double getElbowRotationFinalTargetNativeUnits() {
    return this.elbowRotationFinalTargetNativeUnits;
  }
  public double getElbowRotationInstantaneousTargetNativeUnits() {
    return elbowRotationInstantaneousTargetNativeUnits;
  }
  public void setElbowRotationVoltage(double voltage) {
    elbowRotationMotor.setVoltage(voltage);
  }

  public void setRotationVoltage(double voltage) {
    elbowRotationMotor.setVoltage(voltage);
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
