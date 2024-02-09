package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ravenhardware.BufferedDigitalInput;
import frc.robot.Constants;
import frc.robot.RobotMap;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;



public class shooterSubsystem extends SubsystemBase{
    BufferedDigitalInput pieceSensor = new BufferedDigitalInput(RobotMap.PIECE_SENSOR, 9, false, false);
    CANSparkMax leftSparkMax = new CANSparkMax(0, MotorType.kBrushless);
    CANSparkMax rightSparkMax = new CANSparkMax(0, MotorType.kBrushless);

    public void startShooter(){
        leftSparkMax.set(Constants.SHOOTER_MOTOR_SPEED);
        rightSparkMax.set(Constants.SHOOTER_MOTOR_SPEED);
    }

    public void stopShooting() {
        leftSparkMax.set(0);
        rightSparkMax.set(0);
    }

    public boolean hasPiece(){
        return pieceSensor.get();
    }
}
