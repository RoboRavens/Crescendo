// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.shooter;

/** Add your docs here. */
    public class ShooterSpeedOneSide {
        private double _feedForward;
        private double _targetRotationsPerSecond;
        private double _actualRotationsPerSecond;
        public ShooterSpeedOneSide(double ff, double target, double actual) {
            _feedForward = ff;
            _targetRotationsPerSecond = target;
            _actualRotationsPerSecond = actual;
        }
        public double getFeedForward() {
            return _feedForward;
        }
        public double getTargetRotationsPerSecond() {
            return _targetRotationsPerSecond;
        }
        public double getActualRotationsPerSecond() {
            return _actualRotationsPerSecond;
        }
    }