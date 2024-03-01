package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.ravenhardware.BufferedDigitalInput;
import frc.robot.RobotMap;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.IntakeFeedCommand;
import frc.robot.commands.intake.TrapIntakeCommand;
import frc.robot.commands.intake.TrapLaunchCommand;
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

  public Command createIntakeWithSensorCommand() {
    return createDelayedEndCommand(new IntakeCommand(this), () -> this.trapHasPiece(), 1);
  }

  public Command createFeederWithSensorCommand() {
    return createDelayedEndCommand(new IntakeFeedCommand(this), () -> !this.trapHasPiece(), 1);
  }

  public Command createTrapIntakeWithSensorCommand() {
    return createDelayedEndCommand(new TrapIntakeCommand(this), () -> this.trapHasPiece(), 1);
  }

  public Command createTrapLauncherWithSensorCommand() {
    return createDelayedEndCommand(new TrapLaunchCommand(this), () -> !this.trapHasPiece(), 1);
  }

  public Command createDelayedEndCommand(Command command, BooleanSupplier endCondition, double seconds) {
    var endConditionCommand = new WaitUntilCommand(endCondition);

    return new ParallelRaceGroup(
        command,
        new SequentialCommandGroup(endConditionCommand, new WaitCommand(seconds)));
  }

}
