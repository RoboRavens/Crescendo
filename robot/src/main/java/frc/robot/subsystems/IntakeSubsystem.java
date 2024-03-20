package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ravenhardware.BufferedDigitalInput;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.util.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private boolean _possesesIndexedPiece = true;
  private boolean _finishedIndexingForward = true;

  private BufferedDigitalInput _pieceSensorIntake = new BufferedDigitalInput(RobotMap.INTAKE_INTAKE_SENSOR_DIO_PORT, 3, false,
      false);
  private BufferedDigitalInput _pieceSensorFeeder = new BufferedDigitalInput(RobotMap.INTAKE_FEEDER_SENSOR_DIO_PORT, 3, false,
      false);
  private TalonFX _intakeMotorTop = new TalonFX(RobotMap.INTAKE_MOTOR_TOP_CAN_ID);
  private TalonFX _intakeMotorBottom = new TalonFX(RobotMap.INTAKE_MOTOR_BOTTOM_CAN_ID);
  final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

  public IntakeSubsystem() {

    var config = new TalonFXConfiguration();
    config.MotorOutput.DutyCycleNeutralDeadband = 0.001;
    config.Slot0.kP = 0.15;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    _intakeMotorTop.getConfigurator().apply(config);
    _intakeMotorBottom.getConfigurator().apply(config);
  }
  
  public void startIntake() {
    this.setPowerManually(IntakeConstants.INTAKE_MOTOR_SPEED);
  }

  public void indexPieceForward() {
    this.setPowerManually(IntakeConstants.INTAKE_INDEX_PIECE_FORWARD_MOTOR_SPEED);
  }

  public void indexPieceForwardFast() {
    this.setPowerManually(IntakeConstants.INTAKE_SPEED_BEFORE_FEEDER_SENSOR);
  }

  public void indexPieceBackward() {
    this.setPowerManually(IntakeConstants.INTAKE_INDEX_PIECE_BACKWARD_MOTOR_SPEED * -1);
  }

  public void reverseIntake() {
    this.setPowerManually(IntakeConstants.INTAKE_REVERSE_MOTOR_SPEED);
  }

  public void startIntakeFeeder() {
    this.setPowerManually(IntakeConstants.INTAKE_FEEDER_SPEED);
  }

  public void setPowerManually(double speed) {
    _intakeMotorTop.set(speed);
    _intakeMotorBottom.set(speed);
}

  public void stopMotorWithPID() {
    // _intakeMotor.setControl(m_request.withVelocity(0));
    this.setPowerManually(0);
  }

  // Stops all
  public void stop() {
    this.setPowerManually(IntakeConstants.INTAKE_MOTOR_STOP);
  }

  public void startTrapIntake() {
    this.setPowerManually(IntakeConstants.INTAKE_MOTOR_SPEED);
  }

  public void startTrapLaunch() {
    this.setPowerManually(IntakeConstants.INTAKE_MOTOR_SPEED);
  }

  public boolean intakeHasPiece() {
    return _pieceSensorIntake.get();
  }

  public boolean feederHasPiece() {
    return _pieceSensorFeeder.get();
  }
  

  public void updatePieceIndexer() {
    if (hasPieceAnywhere() == false) {
      clearIndexBooleans();
    }
  }

  public void clearIndexBooleans() {
    setPossesesIndexedPiece(false);
    setFinishedIndexingForward(false);
  }

  public void setPossesesIndexedPiece(boolean hasIndexedPiece) {
    _possesesIndexedPiece = hasIndexedPiece;
  }

  public boolean getPossesesIndexedPiece() {
    return _possesesIndexedPiece;
  }

  public void setFinishedIndexingForward(boolean finishedIndexingForward) {
    _finishedIndexingForward = finishedIndexingForward;
  }

  public boolean getFinishedIndexingForward() {
    return _finishedIndexingForward;
  }

  public boolean hasPieceAnywhere() {
    return this.intakeHasPiece()
      || this.feederHasPiece()
      || Robot.SHOOTER_SUBSYSTEM.hasPiece();
  }

  public boolean driverCanIntake() {
    return this.hasPieceAnywhere() == false;
  }

  @Override
  public void periodic(){
    _pieceSensorIntake.maintainState();
    _pieceSensorFeeder.maintainState();
    SmartDashboard.putBoolean("Intake Piece", this.intakeHasPiece());
    SmartDashboard.putBoolean("Feeder Piece", this.feederHasPiece());
    SmartDashboard.putBoolean("Intake Posesses Index Piece", this.getPossesesIndexedPiece());
    SmartDashboard.putBoolean("Intake Finished Index Forward", this.getFinishedIndexingForward());

    updatePieceIndexer();
  }
}
