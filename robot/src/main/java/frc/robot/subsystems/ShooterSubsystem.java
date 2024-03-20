package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ravenhardware.BufferedDigitalInput;
import frc.robot.RobotMap;
import frc.robot.util.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private BufferedDigitalInput _shooterPieceSensor = new BufferedDigitalInput(RobotMap.SHOOTER_PIECE_SENSOR_DIO_PORT, 3, false,
            false);
    private TalonFX _leftTalonFX = new TalonFX(RobotMap.SHOOTER_LEFT_MOTOR_CAN_ID);
    private TalonFX _rightTalonFX = new TalonFX(RobotMap.SHOOTER_RIGHT_MOTOR_CAN_ID);
    private InterpolatingDoubleTreeMap shooterAngleMapUp = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap shooterAngleMapDown = new InterpolatingDoubleTreeMap();
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);


    public ShooterSubsystem() {
        var leftSlot0Configs = new Slot0Configs();
        leftSlot0Configs.kP = ShooterConstants.lkP;
        leftSlot0Configs.kI = ShooterConstants.lkI;
        leftSlot0Configs.kD = ShooterConstants.lkD;
        _leftTalonFX.getConfigurator().apply(leftSlot0Configs);

        var rightSlot0Configs = new Slot0Configs();
        rightSlot0Configs.kP = ShooterConstants.rkP;
        rightSlot0Configs.kI = ShooterConstants.rkI;
        rightSlot0Configs.kD = ShooterConstants.rkD;
        _rightTalonFX.getConfigurator().apply(rightSlot0Configs);

        var talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Feedback.SensorToMechanismRatio = 1.0 / 3.0;

        _leftTalonFX.getConfigurator().apply(talonFXConfiguration);
        _rightTalonFX.getConfigurator().apply(talonFXConfiguration);
        _leftTalonFX.setNeutralMode(NeutralModeValue.Coast);
        _rightTalonFX.setNeutralMode(NeutralModeValue.Coast);
        _leftTalonFX.setInverted(true);

        _leftTalonFX.getConfigurator().apply(leftSlot0Configs);
        _rightTalonFX.getConfigurator().apply(rightSlot0Configs);
        
        populateShooterAngleMapUp();
        populateShooterAngleMapDown();
    }

    public void runShooterAtTargetSpeed() {
        _leftTalonFX.setControl(m_request
            .withVelocity(ShooterConstants.TARGET_RPS_LEFT)
            .withFeedForward(ShooterConstants.FF_FOR_TARGET_LEFT));
        _rightTalonFX.setControl(m_request
            .withVelocity(ShooterConstants.TARGET_RPS_RIGHT)
            .withFeedForward(ShooterConstants.FF_FOR_TARGET_RIGHT));
    }

    public void setPowerManually(double speed) {
        _leftTalonFX.set(speed);
        _rightTalonFX.set(speed);
    }

    public void stopShooting() {
        _leftTalonFX.set(0);
        _rightTalonFX.set(0);
    }

    public boolean hasPiece() {
        return _shooterPieceSensor.get();
    }

    private void populateShooterAngleMapUp(){
        for(int i=0; i<ShooterConstants.SHOOTER_ANGLE_PAIRS_UP.length; i++){
            shooterAngleMapUp.put(ShooterConstants.SHOOTER_ANGLE_PAIRS_UP[i][0], ShooterConstants.SHOOTER_ANGLE_PAIRS_UP[i][1]);
        }
    }

    private void populateShooterAngleMapDown(){
        for(int i=0; i<ShooterConstants.SHOOTER_ANGLE_PAIRS_DOWN.length; i++){
            shooterAngleMapDown.put(ShooterConstants.SHOOTER_ANGLE_PAIRS_DOWN[i][0], ShooterConstants.SHOOTER_ANGLE_PAIRS_DOWN[i][1]);
        }
    }

    public double getShooterAngleMapUp(double distance){
        double angle = shooterAngleMapUp.get(distance);
        return angle;
    }

    public double getShooterAngleMapDown(double distance){
        double angle = shooterAngleMapDown.get(distance);
        return angle;
    }

    public boolean shooterUpToSpeed() {
        return this.leftShooterAtSpeed() && this.rightShooterAtSpeed();
    }

    private boolean leftShooterAtSpeed() {
        double targetSpeed = ShooterConstants.ACTUAL_PID_RPS_FOR_SOME_REASON_LEFT;
        double currentLeftSpeed = this.getLeftRpm();
        double absDiff = Math.abs(targetSpeed - currentLeftSpeed);
        return absDiff < ShooterConstants.IS_AT_TARGET_SPEED_BUFFER;
    }

    private boolean rightShooterAtSpeed() {
        double targetSpeed = ShooterConstants.ACTUAL_PID_RPS_FOR_SOME_REASON_RIGHT;
        double currentRightSpeed = this.getRightRpm();
        double absDiff = Math.abs(targetSpeed - currentRightSpeed);
        return absDiff < ShooterConstants.IS_AT_TARGET_SPEED_BUFFER;
    }

    private double getLeftRpm() {
        return _leftTalonFX.getVelocity().getValueAsDouble();
    }

    private double getRightRpm() {
        return _rightTalonFX.getVelocity().getValueAsDouble();
    }

    public double getShootingAngleFormula(double distance, double shooterHeightMeters) {
        double denumerator = 2 * ShooterConstants.GRAVITY_ACCELERATION * Math.pow(distance, 2);
        double numerator1 = 2 * Math.pow(ShooterConstants.INITIAL_NOTE_SPEED, 2) * distance;
        double numerator2 = Math.pow((2 * Math.pow(ShooterConstants.INITIAL_NOTE_SPEED, 2) * distance), 2);
        double numerator3 = -4 * ShooterConstants.GRAVITY_ACCELERATION * Math.pow(distance, 2);
        double numerator4 = (ShooterConstants.GRAVITY_ACCELERATION * Math.pow(distance, 2) + 2* Math.pow(ShooterConstants.INITIAL_NOTE_SPEED, 2) * (ShooterConstants.SPEAKER_HEIGHT_METERS - shooterHeightMeters));
        double numerator = numerator1 - Math.sqrt(numerator2 + numerator3 * numerator4);
        double shootingAngle = Math.toDegrees(Math.atan(numerator / denumerator));
        return shootingAngle;
    }

  @Override
  public void periodic(){
    _shooterPieceSensor.maintainState();
    SmartDashboard.putBoolean("Shooter Piece", this.hasPiece());

    double leftMaxSpeed = SmartDashboard.getNumber("Shooter Max Left Speed", 0);
    double rightMaxSpeed = SmartDashboard.getNumber("Shooter Max Right Speed", 0);
    leftMaxSpeed = Math.max(leftMaxSpeed, Math.abs(_leftTalonFX.getVelocity().getValueAsDouble()));
    rightMaxSpeed = Math.max(rightMaxSpeed, Math.abs(_rightTalonFX.getVelocity().getValueAsDouble()));
    SmartDashboard.putNumber("Shooter Max Left Speed", leftMaxSpeed);
    SmartDashboard.putNumber("Shooter Max Right Speed", rightMaxSpeed);

    SmartDashboard.putNumber("Shooter Target Left Speed", ShooterConstants.TARGET_RPS_LEFT);
    SmartDashboard.putNumber("Shooter Target Right Speed", ShooterConstants.TARGET_RPS_RIGHT);

    double currentLeftSpeed = this.getLeftRpm();
    double currentRightSpeed = this.getRightRpm();
    SmartDashboard.putNumber("Shooter Current Left Speed", currentLeftSpeed);
    SmartDashboard.putNumber("Shooter Current Right Speed", currentRightSpeed);
    
    SmartDashboard.putNumber("Shooter Target Left Diff", ShooterConstants.TARGET_RPS_LEFT - currentLeftSpeed);
    SmartDashboard.putNumber("Shooter Target Right Diff", ShooterConstants.TARGET_RPS_RIGHT - currentRightSpeed);

    SmartDashboard.putBoolean("Shooter Left At Speed", this.leftShooterAtSpeed());
    SmartDashboard.putBoolean("Shooter Right At Speed", this.rightShooterAtSpeed());
  }
}
