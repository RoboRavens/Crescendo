package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ravenhardware.BufferedDigitalInput;
import frc.robot.Constants;

public class IntakeTrapSubsystem extends SubsystemBase{

    private BufferedDigitalInput pieceSensorIntake = new BufferedDigitalInput(RobotMap.PIECE_SENSOR_INTAKE, 3, false,
            false);
    private BufferedDigitalInput pieceSensorTrap = new BufferedDigitalInput(RobotMap.PIECE_SENSOR_TRAP, 3, false,
            false);
    private CANSparkMax sparkMax = new CANSparkMax(0, MotorType.kBrushless);

    public void startIntake() {
        sparkMax.set(Constants.INTAKE_SPARK_MAX_SPEED);

    }

    public void stopIntake() {
        sparkMax.set(Constants.INTAKE_SPARK_MAX_STOP);
    }

    public void startTrap() {
        sparkMax.set(Constants.INTAKE_SPARK_MAX_STOP);
    }

    public void stopTrap() {
        sparkMax.set(Constants.INTAKE_SPARK_MAX_STOP);
    }

    public boolean intakeHasPiece() {
        return pieceSensorIntake.get();
    }

    public boolean trapHasPiece() {
        return pieceSensorTrap.get();
    }
}
