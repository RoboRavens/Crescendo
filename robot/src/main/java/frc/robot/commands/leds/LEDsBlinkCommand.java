// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class LEDsBlinkCommand extends Command {
  private int _r;
  private int _g;
  private int _b;

  /** Creates a new LEDsBlinkCommand. */
  public LEDsBlinkCommand(int r, int g, int b) {
    _r = r;
    _g = g;
    _b = b;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.ledsSubsystem24);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Started led blink");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.ledsSubsystem24.ledsBlinkColors(_r, _g, _b, 0, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.ledsSubsystem24.ledsSolidColor(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
