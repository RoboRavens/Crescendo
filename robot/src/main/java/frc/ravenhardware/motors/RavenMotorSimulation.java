package frc.ravenhardware.motors;

public class RavenMotorSimulation implements RavenMotor {
  private RavenMotorSimulationConfig _config;

  private boolean _usePID = false;
  private double _p = 0;

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

  public void setPID(double p, double i, double d) {
    _p = p;
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
  public void setTargetPosition(double targetPosition) {
    _targetPosition = targetPosition;
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
    if (_usePID) {
      _position += _config.PowerToPositionRatio * (_targetPosition - _position) * _p;
    } else {
      _position += _config.PowerToPositionRatio * _power;
    }
  }
}
