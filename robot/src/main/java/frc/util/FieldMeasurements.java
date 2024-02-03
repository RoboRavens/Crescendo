package frc.util;

import edu.wpi.first.math.geometry.Translation2d;

public class FieldMeasurements {
    public static final Translation2d[] BLUE_SOURCES = {
        new Translation2d(15.88, 1.34), // Left source from the driver's perspective
        new Translation2d(15.36, 1.02), // Middle source from the driver's perspective
        new Translation2d(14.8, 0.66) // Right source from the driver's perspective
    };
    public static final Translation2d[] RED_SOURCES = {
        new Translation2d(1.72, 0.68), // Left source from the driver's perspective
        new Translation2d(1.14, 1), // Middle source from the driver's perspective
        new Translation2d(0.58, 1.33) // Right source from the driver's perspective
    };
    public static final Translation2d BLUE_AMP = new Translation2d(1.8, 7.75);
    public static final Translation2d RED_AMP = new Translation2d(14.7, 7.75);
}
