// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.WristSubsystem;

/** Add your docs here. */
public class WristSubsystemTests {
        @Test 
    public void positionCalculationTest(){
        double testValue = -20;
        double rads = WristSubsystem.getRadiansFromPosition(testValue);
        double pos = WristSubsystem.getPositionFromRadians(rads);
        
        assertEquals(testValue, pos);
    }
}
