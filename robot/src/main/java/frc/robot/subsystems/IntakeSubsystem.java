package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ravenhardware.BufferedDigitalInput;
import frc.robot.RobotMap;
import frc.robot.util.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private BufferedDigitalInput _pieceSensorIntake = new BufferedDigitalInput(RobotMap.INTAKE_INTAKE_SENSOR_DIO_PORT, 3, false,
      false);
  private BufferedDigitalInput _pieceSensorTrap = new BufferedDigitalInput(RobotMap.INTAKE_TRAP_SENSOR_DIO_PORT, 3, false,
      false);
  private TalonFX _intakeMotor = new TalonFX(RobotMap.INTAKE_MOTOR_CAN_ID);

  public void startIntake() {
    _intakeMotor.set(IntakeConstants.INTAKE_SPARK_MAX_SPEED);
  }

  public void setPowerManually(double speed) {
    _intakeMotor.set(speed);
}

  // Stops all
  public void stop() {
    _intakeMotor.set(IntakeConstants.INTAKE_SPARK_MAX_STOP);
  }

  public void startTrapIntake() {
    _intakeMotor.set(IntakeConstants.INTAKE_SPARK_MAX_SPEED * -1);
  }

  public void startTrapLaunch() {
    _intakeMotor.set(IntakeConstants.INTAKE_SPARK_MAX_SPEED);
  }

  public boolean intakeHasPiece() {
    return _pieceSensorIntake.get();
  }

  public boolean trapHasPiece() {
    return _pieceSensorTrap.get();
  }
}
