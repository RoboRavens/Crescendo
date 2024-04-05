package frc.ravenhardware.motors;

public interface RavenMotor {
  void set(double power);
  double get();
  void setPosition(double position);
  void setTargetPosition(double position);
  double getTargetPosition();
  double getPosition();
  void periodic();
}