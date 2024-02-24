package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.ravenhardware.BufferedDigitalInput;
import frc.robot.RobotMap;
import frc.robot.commands.shooter.ShootCommand;

public class ShooterSubsystem extends SubsystemBase {
    private BufferedDigitalInput _shooterPieceSensor = new BufferedDigitalInput(RobotMap.SHOOTER_PIECE_SENSOR, 3, false,
            false);
    private CANSparkMax _leftSparkMax = new CANSparkMax(RobotMap.SHOOTER_LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
    private CANSparkMax _rightSparkMax = new CANSparkMax(RobotMap.SHOOTER_RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
    private SparkPIDController _lSparkPIDController = _leftSparkMax.getPIDController();
    private SparkPIDController _rSparkPIDController = _rightSparkMax.getPIDController();
    private InterpolatingDoubleTreeMap shooterAngleMap = new InterpolatingDoubleTreeMap();

    public ShooterSubsystem() {
        _lSparkPIDController.setP(ShooterConstants.lkP);
        _lSparkPIDController.setI(ShooterConstants.lkI);
        _lSparkPIDController.setD(ShooterConstants.lkD);
        _lSparkPIDController.setIZone(ShooterConstants.lkIz);
        _lSparkPIDController.setFF(ShooterConstants.lkFF);
        _lSparkPIDController.setOutputRange(ShooterConstants.lkMinOutput, ShooterConstants.lkMaxOutput);

        _rSparkPIDController.setP(ShooterConstants.rkP);
        _rSparkPIDController.setI(ShooterConstants.rkI);
        _rSparkPIDController.setD(ShooterConstants.rkD);
        _rSparkPIDController.setIZone(ShooterConstants.rkIz);
        _rSparkPIDController.setFF(ShooterConstants.rkFF);
        _rSparkPIDController.setOutputRange(ShooterConstants.rkMinOutput, ShooterConstants.rkMaxOutput);

        populateShooterAngleMap();
    }

    public void startShooter() {
        _lSparkPIDController.setReference(ShooterConstants.lmaxRPM * ShooterConstants.lShooterVelocityPercentage, CANSparkMax.ControlType.kVelocity);
        _rSparkPIDController.setReference(ShooterConstants.rmaxRPM * ShooterConstants.rShooterVelocityPercentage, CANSparkMax.ControlType.kVelocity);
    }

    public void stopShooting() {
        _leftSparkMax.set(0);
        _rightSparkMax.set(0);
    }

    public boolean hasPiece() {
        return _shooterPieceSensor.get();
    }

    private void populateShooterAngleMap(){
        for(int i=0; i<ShooterConstants.SHOOTER_ANGLE_PAIRS.length; i++){
            shooterAngleMap.put(ShooterConstants.SHOOTER_ANGLE_PAIRS[i][0], ShooterConstants.SHOOTER_ANGLE_PAIRS[i][1]);
        }
    }

    public double getShooterAngleMap(double distance){
        double angle = shooterAngleMap.get(distance);
        return angle;
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
