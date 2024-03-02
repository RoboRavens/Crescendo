package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ravenhardware.BufferedDigitalInput;
import frc.robot.RobotMap;
import frc.robot.util.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private BufferedDigitalInput _shooterPieceSensor = new BufferedDigitalInput(RobotMap.SHOOTER_PIECE_SENSOR, 3, false,
            false);
    private TalonFX _leftTalonFX = new TalonFX(RobotMap.SHOOTER_LEFT_MOTOR_CAN_ID);
    private TalonFX _rightTalonFX = new TalonFX(RobotMap.SHOOTER_RIGHT_MOTOR_CAN_ID);
    private InterpolatingDoubleTreeMap shooterAngleMapUp = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap shooterAngleMapDown = new InterpolatingDoubleTreeMap();
    private double _leftTargetSpeed;
    private double _rightTargetSpeed;
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    boolean isOn = false;


    public ShooterSubsystem() {
        
        var leftSlot0Configs = new Slot0Configs();
        leftSlot0Configs.kS = ShooterConstants.lkS;
        leftSlot0Configs.kV = ShooterConstants.lkV;
        leftSlot0Configs.kP = ShooterConstants.lkP;
        leftSlot0Configs.kI = ShooterConstants.lkI;
        leftSlot0Configs.kD = ShooterConstants.lkD;
        _leftTalonFX.getConfigurator().apply(leftSlot0Configs);

        var rightSlot0Configs = new Slot0Configs();
        rightSlot0Configs.kS = ShooterConstants.rkS;
        rightSlot0Configs.kV = ShooterConstants.rkV;
        rightSlot0Configs.kP = ShooterConstants.rkP;
        rightSlot0Configs.kI = ShooterConstants.rkI;
        rightSlot0Configs.kD = ShooterConstants.rkD;
        _rightTalonFX.getConfigurator().apply(rightSlot0Configs);
        
        populateShooterAngleMapUp();
        populateShooterAngleMapDown();
    }

    public void runShooterAtTargetSpeed() {
        if(isOn == false){
            stopShooting();
        }
        else{
            _leftTalonFX.setControl(m_request.withVelocity(_leftTargetSpeed).withFeedForward(ShooterConstants.LEFT_FEED_FORWARD));
            _rightTalonFX.setControl(m_request.withVelocity(_rightTargetSpeed).withFeedForward(ShooterConstants.RIGHT_FEED_FORWARD));
        } 
    }

    public void setPowerManually(double speed) {
        _leftTalonFX.set(speed);
        _rightTalonFX.set(speed * -1);
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

    public double getLeftTargetSpeed() {
        return _leftTargetSpeed;
    }

    public void setLeftTargetSpeed(double _leftTargetSpeed) {
        this._leftTargetSpeed = _leftTargetSpeed;
    }

    public double getRightTargetSpeed() {
        return _rightTargetSpeed;
    }

    public void setRightTargetSpeed(double _rightTargetSpeed) {
        this._rightTargetSpeed = _rightTargetSpeed;
    }

    public boolean getIsOn(){
        return isOn;
    }

    public void setIsOn(boolean isOn){
        this.isOn = isOn;
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
}
