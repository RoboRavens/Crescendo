// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class LEDsBlinkCommand extends Command {
  private double blinkDurationSeconds;
  private int red;
  private int green;
  private int blue;
  private int red2;
  private int green2;
  private int blue2;
  private Timer timer = new Timer();
  private boolean blinkOn = false;

  /** Creates a new LEDsBlinkCommand. */
  public LEDsBlinkCommand(Color color, Color color2, double blinkDurationSeconds) {
    this.blinkDurationSeconds = blinkDurationSeconds;
    this.red = (int)color.red * 255;
    this.green = (int)color.green * 255;
    this.blue = (int)color.blue * 255;
    addRequirements(Robot.ledsSubsystem24);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    System.out.println("Started led blink");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() >= blinkDurationSeconds) {
      // toggle state
      blinkOn = !blinkOn;
      timer.reset();
      timer.start();
    }

    if (blinkOn) {
      Robot.ledsSubsystem24.ledsSolidColor(red, green, blue);
    } 
    else{
      Robot.ledsSubsystem24.ledsSolidColor(red2, green2, blue2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.ledsSubsystem24.ledsSolidColor(0, 0, 0);
    System.out.println("Ended led blink");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
