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
import frc.robot.util.shooter.ShooterSpeed;

public class ShooterSubsystem extends SubsystemBase {
    private BufferedDigitalInput _shooterPieceSensor = new BufferedDigitalInput(RobotMap.SHOOTER_PIECE_SENSOR_DIO_PORT, 3, false,
            false);
    private TalonFX _leftTalonFX = new TalonFX(RobotMap.SHOOTER_LEFT_MOTOR_CAN_ID);
    private TalonFX _rightTalonFX = new TalonFX(RobotMap.SHOOTER_RIGHT_MOTOR_CAN_ID);
    private InterpolatingDoubleTreeMap shooterAngleMapUp = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap shooterAngleMapDown = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap shooterAngleLLTyMapUp = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap shooterAngleLLTyMapDown = new InterpolatingDoubleTreeMap();
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    private int _shooterSpeed = 0;
    private boolean _isShooting = false;

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
        populateShooterAngleLLTyMapUp();
        populateShooterAngleLLTyMapDown();
    }

    public void toggleShooterSpeeds() {
        _shooterSpeed += 1;
        _shooterSpeed = _shooterSpeed % ShooterConstants.SHOOTER_SPEEDS.length;
        if (_isShooting) {
            this.runShooterAtTargetSpeed();
        }
    }

    public void runShooterAtTargetSpeed() {
        _isShooting = true;
        var speedConfig = this.getSpeedConfig();
        _leftTalonFX.setControl(m_request
            .withVelocity(speedConfig.getLeft().getTargetRotationsPerSecond())
            .withFeedForward(speedConfig.getLeft().getFeedForward()));
        _rightTalonFX.setControl(m_request
            .withVelocity(speedConfig.getRight().getTargetRotationsPerSecond())
            .withFeedForward(speedConfig.getRight().getFeedForward()));
    }

    public void setPowerManually(double speed) {
        _isShooting = false;
        _leftTalonFX.set(speed);
        _rightTalonFX.set(speed);
    }

    public void stopShooting() {
        _isShooting = false;
        _leftTalonFX.set(0);
        _rightTalonFX.set(0);
    }

    public boolean hasPiece() {
        return _shooterPieceSensor.get();
    }

    private ShooterSpeed getSpeedConfig() {
        return ShooterConstants.SHOOTER_SPEEDS[_shooterSpeed];
    }

    private void populateShooterAngleMapUp(){
      var array = ShooterConstants.SHOOTER_ANGLE_PAIRS_UP;
      for(int i = 0; i < array.length; i++){
          shooterAngleMapUp.put(array[i][0], array[i][1]);
      }
    }

    private void populateShooterAngleMapDown(){
      var array = ShooterConstants.SHOOTER_ANGLE_PAIRS_DOWN;
      for(int i = 0; i < array.length; i++){
          shooterAngleMapDown.put(array[i][0], array[i][1]);
      }
    }

    private void populateShooterAngleLLTyMapUp(){
      var array = ShooterConstants.SHOOTER_ANGLE_FROM_TY_UP;
      for(int i = 0; i < array.length; i++){
          shooterAngleLLTyMapUp.put(array[i][0], array[i][1]);
      }
    }

    private void populateShooterAngleLLTyMapDown(){
      var array = ShooterConstants.SHOOTER_ANGLE_FROM_TY_DOWN;
      for(int i = 0; i < array.length; i++){
          shooterAngleLLTyMapDown.put(array[i][0], array[i][1]);
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

    public double getShooterAngleLLTyMapUp(double distance){
        double angle = shooterAngleLLTyMapUp.get(distance);
        return angle;
    }

    public double getShooterAngleLLTyMapDown(double distance){
        double angle = shooterAngleLLTyMapDown.get(distance);
        return angle;
    }

    public boolean shooterUpToSpeed() {
        return this.leftShooterAtSpeed() && this.rightShooterAtSpeed();
    }

    private boolean leftShooterAtSpeed() {
        double targetSpeed = this.getSpeedConfig().getLeft().getActualRotationsPerSecond();
        double currentLeftSpeed = this.getLeftRpm();
        double absDiff = Math.abs(targetSpeed - currentLeftSpeed);
        return absDiff < ShooterConstants.IS_AT_TARGET_SPEED_BUFFER;
    }

    private boolean rightShooterAtSpeed() {
        double targetSpeed = this.getSpeedConfig().getRight().getActualRotationsPerSecond();
        double currentRightSpeed = this.getRightRpm();
        double absDiff = Math.abs(targetSpeed - currentRightSpeed);
        return absDiff < ShooterConstants.IS_AT_TARGET_SPEED_BUFFER;
    }

    public double getLeftRpm() {
        return _leftTalonFX.getVelocity().getValueAsDouble();
    }

    public double getRightRpm() {
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

    var speedConfig = this.getSpeedConfig();
    SmartDashboard.putString("Speed Config", speedConfig.getName());

    SmartDashboard.putNumber("Shooter Target Left Speed", speedConfig.getLeft().getTargetRotationsPerSecond());
    SmartDashboard.putNumber("Shooter Target Right Speed", speedConfig.getRight().getTargetRotationsPerSecond());

    double currentLeftSpeed = this.getLeftRpm();
    double currentRightSpeed = this.getRightRpm();
    SmartDashboard.putNumber("Shooter Current Left Speed", currentLeftSpeed);
    SmartDashboard.putNumber("Shooter Current Right Speed", currentRightSpeed);
    
    SmartDashboard.putNumber("Shooter Target Left Diff", speedConfig.getLeft().getTargetRotationsPerSecond() - currentLeftSpeed);
    SmartDashboard.putNumber("Shooter Target Right Diff", speedConfig.getRight().getTargetRotationsPerSecond() - currentRightSpeed);

    SmartDashboard.putBoolean("Shooter Left At Speed", this.leftShooterAtSpeed());
    SmartDashboard.putBoolean("Shooter Right At Speed", this.rightShooterAtSpeed());
  }
}
