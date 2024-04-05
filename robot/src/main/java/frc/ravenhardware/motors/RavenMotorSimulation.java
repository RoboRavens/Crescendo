package frc.ravenhardware.motors;

public class RavenMotorSimulation implements RavenMotor {
  private RavenMotorSimulationConfig _config;
  private boolean _usePID = false;
  private double _power = 0;
  private double _position = 0;
  private double _targetPosition = 0;

  public RavenMotorSimulation(RavenMotorSimulationConfig config) {
    _config = config;
  }

  @Override
  public void set(double power) {
    _power = power;
  }

  @Override
  public double get() {
    return _power;
  }

  @Override
  public void setPosition(double position) {
    _position = position;
  }

  @Override
  public void setTargetPosition(double position) {
    _targetPosition = 0;
  }

  @Override
  public double getTargetPosition() {
    return _targetPosition;
  }

  @Override
  public double getPosition() {
    return _position;
  }

  @Override
  public void periodic() {

  }
}
