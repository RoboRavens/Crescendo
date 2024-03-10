package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ravenhardware.BufferedDigitalInput;
import frc.robot.RobotMap;
import frc.robot.util.Constants.IntakeConstants;
import frc.robot.util.Constants.ShooterConstants;

public class IntakeSubsystem extends SubsystemBase {

  private BufferedDigitalInput _pieceSensorIntake = new BufferedDigitalInput(RobotMap.INTAKE_INTAKE_SENSOR_DIO_PORT, 3, false,
      false);
  private BufferedDigitalInput _pieceSensorTrap = new BufferedDigitalInput(RobotMap.INTAKE_TRAP_SENSOR_DIO_PORT, 3, false,
      false);
  private TalonFX _intakeMotor = new TalonFX(RobotMap.INTAKE_MOTOR_CAN_ID);
  final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

  public IntakeSubsystem() {
    var leftSlot0Configs = new Slot0Configs();
        leftSlot0Configs.kP = 0.15;
    _intakeMotor.getConfigurator().apply(leftSlot0Configs);
    _intakeMotor.setNeutralMode(NeutralModeValue.Brake);
  }
  
  public void startIntake() {
    _intakeMotor.set(IntakeConstants.INTAKE_MOTOR_SPEED * -1);
  }

  public void reverseIntake() {
    _intakeMotor.set(IntakeConstants.INTAKE_MOTOR_SPEED);
  }

  public void startIntakeFeeder() {
    _intakeMotor.set(IntakeConstants.INTAKE_FEEDER_SPEED * -1);
  }

  public void setPowerManually(double speed) {
    _intakeMotor.set(speed);
}

  public void stopMotorWithPID() {
    // _intakeMotor.setControl(m_request.withVelocity(0));
    _intakeMotor.set(0);
  }

  // Stops all
  public void stop() {
    _intakeMotor.set(IntakeConstants.INTAKE_MOTOR_STOP);
  }

  public void startTrapIntake() {
    _intakeMotor.set(IntakeConstants.INTAKE_MOTOR_SPEED);
  }

  public void startTrapLaunch() {
    _intakeMotor.set(IntakeConstants.INTAKE_MOTOR_SPEED);
  }

  public boolean intakeHasPiece() {
    return _pieceSensorIntake.get();
  }

  public boolean trapHasPiece() {
    return _pieceSensorTrap.get();
  }

  @Override
  public void periodic(){
    _pieceSensorIntake.maintainState();
    _pieceSensorTrap.maintainState();
    SmartDashboard.putBoolean("Intake Piece", this.intakeHasPiece());
    SmartDashboard.putBoolean("Trap Piece", this.trapHasPiece());
  }
}
