// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.shooter;

/** Add your docs here. */
public class ShooterSpeed {
  private String _name;
  private ShooterSpeedOneSide _left;
  private ShooterSpeedOneSide _right;

  public ShooterSpeed(String name, ShooterSpeedOneSide left, ShooterSpeedOneSide right) {
    _name = name;
    _left = left;
    _right = right;
  }

  public String getName() {
    return _name;
  }

  public ShooterSpeedOneSide getLeft() {
    return _left;
  }

  public ShooterSpeedOneSide getRight() {
    return _right;
  }
}
