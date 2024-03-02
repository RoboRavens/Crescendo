// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.field;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class FieldConstants {
    public static final double BLUE_AMP_ZONE_SOUTHWEST_CORNER_X_METERS = 0;
    public static final double BLUE_AMP_ZONE_SOUTHWEST_CORNER_Y_METERS = 7.8;
    public static final double BLUE_AMP_ZONE_WIDTH_METERS = 3.26;
    public static final double BLUE_AMP_ZONE_HEIGHT_METERS = 0.44;

    public static final double BLUE_STAGE_ZONE_SOUTHWEST_CORNER_X_METERS = 3.14;
    public static final double BLUE_STAGE_ZONE_SOUTHWEST_CORNER_Y_METERS = 2.55;
    public static final double BLUE_STAGE_ZONE_WIDTH_METERS = 2.71;
    public static final double BLUE_STAGE_ZONE_HEIGHT_METERS = 3.17;

    public static final double BLUE_ROBOT_STARTING_ZONE_SOUTHWEST_CORNER_X_METERS = 0;
    public static final double BLUE_ROBOT_STARTING_ZONE_SOUTHWEST_CORNER_Y_METERS = 1.61;
    public static final double BLUE_ROBOT_STARTING_ZONE_WIDTH_METERS = 1.9;
    public static final double BLUE_ROBOT_STARTING_ZONE_HEIGHT_METERS = 6.19;

    public static final double BLUE_SOURCE_ZONE_SOUTHWEST_CORNER_X_METERS = 0;
    public static final double BLUE_SOURCE_ZONE_SOUTHWEST_CORNER_Y_METERS = 0;
    public static final double BLUE_SOURCE_ZONE_WIDTH_METERS = 1.9;
    public static final double BLUE_SOURCE_ZONE_HEIGHT_METERS = 1.61;

    public static final double BLUE_WING_ZONE_1_SOUTHWEST_CORNER_X_METERS = 3.26;
    public static final double BLUE_WING_ZONE_1_SOUTHWEST_CORNER_Y_METERS = 7.8;
    public static final double BLUE_WING_ZONE_1_WIDTH_METERS = 2.59;
    public static final double BLUE_WING_ZONE_1_HEIGHT_METERS = 0.44;

    public static final double BLUE_WING_ZONE_2_ZONE_SOUTHWEST_CORNER_X_METERS = 1.9;
    public static final double BLUE_WING_ZONE_2_SOUTHWEST_CORNER_Y_METERS = 5.72;
    public static final double BLUE_WING_ZONE_2_WIDTH_METERS = 3.95;
    public static final double BLUE_WING_ZONE_2_HEIGHT_METERS = 2.08;

    public static final double BLUE_WING_ZONE_3_SOUTHWEST_CORNER_X_METERS = 1.9;
    public static final double BLUE_WING_ZONE_3_SOUTHWEST_CORNER_Y_METERS = 2.55;
    public static final double BLUE_WING_ZONE_3_WIDTH_METERS = 1.24;
    public static final double BLUE_WING_ZONE_3_HEIGHT_METERS = 3.17;

    public static final double BLUE_WING_ZONE_4_SOUTHWEST_CORNER_X_METERS = 1.9;
    public static final double BLUE_WING_ZONE_4_SOUTHWEST_CORNER_Y_METERS = 0;
    public static final double BLUE_WING_ZONE_4_WIDTH_METERS = 3.95;
    public static final double BLUE_WING_ZONE_4_HEIGHT_METERS = 2.55;
    
    public static final double WEST_NEUTRAL_ZONE_SOUTHWEST_CORNER_X_METERS = 5.85;
    public static final double WEST_NEUTRAL_ZONE_SOUTHWEST_CORNER_Y_METERS = 0;
    public static final double WEST_NEUTRAL_ZONE_WIDTH_METERS = 2.42;
    public static final double WEST_NEUTRAL_ZONE_HEIGHT_METERS = 8.23;

    public static final double EAST_NEUTRAL_ZONE_SOUTHWEST_CORNER_X_METERS = 8.27;
    public static final double EAST_NEUTRAL_ZONE_SOUTHWEST_CORNER_Y_METERS = 0;
    public static final double EAST_NEUTRAL_ZONE_WIDTH_METERS = 2.42;
    public static final double EAST_NEUTRAL_ZONE_HEIGHT_METERS = 8.23;

    // Note: the below positions are for the blue side. Use the below methods to transform to red
    // blue: opp left --> red: opp right
    public static final Translation2d CLIMBING_POINT_1_METERS = new Translation2d(6.31, 5.01); 
    public static final double CLIMBING_POINT_1_ROTATION_RADIAN = -3.14;

    // blue: opp center --> red: opp center
    public static final Translation2d CLIMBING_POINT_2_METERS = new Translation2d(6.31, 4.14); 
    public static final double CLIMBING_POINT_2_ROTATION_RADIAN = -3.14;

    // blue: opp right --> red: opp left
    public static final Translation2d CLIMBING_POINT_3_METERS = new Translation2d(6.31, 3.26); 
    public static final double CLIMBING_POINT_3_ROTATION_RADIAN = -3.14;
    
    // blue: right far --> red: left far
    public static final Translation2d CLIMBING_POINT_4_METERS = new Translation2d(4.87, 2.44); 
    public static final double CLIMBING_POINT_4_ROTATION_RADIAN = 1.05;
    
    // blue: right center --> red: left center
    public static final Translation2d CLIMBING_POINT_5_METERS = new Translation2d(4.11, 2.88); 
    public static final double CLIMBING_POINT_5_ROTATION_RADIAN = 1.05;
    
    // blue: right close --> red: left close
    public static final Translation2d CLIMBING_POINT_6_METERS = new Translation2d(3.35, 3.31); 
    public static final double CLIMBING_POINT_6_ROTATION_RADIAN = 1.05;
    
    // blue: left close --> red: right close
    public static final Translation2d CLIMBING_POINT_7_METERS = new Translation2d(3.39, 4.93); 
    public static final double CLIMBING_POINT_7_ROTATION_RADIAN = -1.05;
    
    // blue: left center --> red: right center
    public static final Translation2d CLIMBING_POINT_8_METERS = new Translation2d(4.15, 5.37);
    public static final double CLIMBING_POINT_8_ROTATION_RADIAN = -1.05;
    
    // blue: left far --> red: right far
    public static final Translation2d CLIMBING_POINT_9_METERS = new Translation2d(4.91, 5.8);
    public static final double CLIMBING_POINT_9_ROTATION_RADIAN = -1.05;

    public static final Translation2d BLUE_AMP_SCORING_POSITION = new Translation2d(1.91, 7.74); 
    public static final double BLUE_AMP_SCORING_ROTATION = 1.57;

    public static final Translation2d RED_SOURCE_RIGHT_POSITION = new Translation2d(0.6, 1.36);
    public static final Translation2d RED_SOURCE_CENTER_POSITION = new Translation2d(1.17, 1.03);
    public static final Translation2d RED_SOURCE_LEFT_POSITION = new Translation2d(1.74, 0.7);
    public static final double RED_SOURCE_ROTATION = -2.08;
    
    public static final double FIELD_WIDTH_METERS = 16.54;

    public static Translation2d mirrorPointToRight(Translation2d point) {
        return new Translation2d(FIELD_WIDTH_METERS - point.getX(), point.getY());
    }

    public static double mirrorRotationToRight(double rotationValue) {
        return Math.PI - rotationValue;
    }
}
