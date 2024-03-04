// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.util.Constants.ElbowConstants;
import frc.robot.util.arm.LimbSetpoint;

/** Add your docs here. */
public class ElbowSubsystemTests {
        @Test 
    public void positionCalculationTest(){
        double testValue = -20;
        double rads = ElbowSubsystem.getRadiansFromPosition(testValue);
        double pos = ElbowSubsystem.getPositionFromRadians(rads);
        
        assertEquals(testValue, pos);
    }

    @Test
    public void goToStartPositionTest(){
        var setpoint = new LimbSetpoint("", ElbowConstants.DEGREES_START_CONFIG, 0);

        assertEquals(-23.078, setpoint.getElbowRotationPosition());
    }
}
