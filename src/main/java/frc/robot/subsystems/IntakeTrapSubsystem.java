package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

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

public class IntakeTrapSubsystem extends SubsystemBase {

  private BufferedDigitalInput _pieceSensorIntake = new BufferedDigitalInput(RobotMap.PIECE_SENSOR_INTAKE, 3, false,
      false);
  private BufferedDigitalInput _pieceSensorTrap = new BufferedDigitalInput(RobotMap.PIECE_SENSOR_TRAP, 3, false,
      false);
  private CANSparkMax _sparkMax = new CANSparkMax(0, MotorType.kBrushless);

  public void startIntake() {
    _sparkMax.set(IntakeTrapConstants.INTAKE_SPARK_MAX_SPEED);
  }

  // Stops all
  public void stop() {
    _sparkMax.set(IntakeTrapConstants.INTAKE_SPARK_MAX_STOP);
  }

  public void startTrapIntake() {
    _sparkMax.set(IntakeTrapConstants.INTAKE_SPARK_MAX_SPEED * -1);
  }

  public void startTrapLaunch() {
    _sparkMax.set(IntakeTrapConstants.INTAKE_SPARK_MAX_SPEED);
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
