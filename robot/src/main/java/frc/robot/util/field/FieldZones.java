package frc.robot.util.field;

import java.util.ArrayList;

import java.awt.geom.Point2D;


public class FieldZones {
    public enum FieldMacroZone {
        NONE,
        WING,
        NEUTRAL
    }

    private ArrayList<FieldZone> fieldZones = new ArrayList<FieldZone>();
    private FieldZone blueWing;
    private FieldZone redWing;
    private FieldZone neutralZone;    
    
    // We'll include a none state in case odometry ever breaks by enough that no zone is returned.
    private FieldZone noneZone;

    public static final FieldSubzone westNeutralZone = new FieldSubzone(
        "West Neutral Zone",
        FieldConstants.WEST_NEUTRAL_ZONE_SOUTHWEST_CORNER_X_METERS,
        FieldConstants.WEST_NEUTRAL_ZONE_SOUTHWEST_CORNER_Y_METERS,
        FieldConstants.WEST_NEUTRAL_ZONE_WIDTH_METERS,
        FieldConstants.WEST_NEUTRAL_ZONE_HEIGHT_METERS);

    public static final FieldSubzone eastNeutralZone = new FieldSubzone(
        "East Neutral Zone",
        FieldConstants.EAST_NEUTRAL_ZONE_SOUTHWEST_CORNER_X_METERS,
        FieldConstants.EAST_NEUTRAL_ZONE_SOUTHWEST_CORNER_Y_METERS,
        FieldConstants.EAST_NEUTRAL_ZONE_WIDTH_METERS,
        FieldConstants.EAST_NEUTRAL_ZONE_HEIGHT_METERS);

    public static final MirroredSubzone mirroredAmpSubzone = new MirroredSubzone(
        "Amp Subzone",
        FieldConstants.BLUE_AMP_ZONE_SOUTHWEST_CORNER_X_METERS,
        FieldConstants.BLUE_AMP_ZONE_SOUTHWEST_CORNER_Y_METERS,
        FieldConstants.BLUE_AMP_ZONE_WIDTH_METERS,
        FieldConstants.BLUE_AMP_ZONE_HEIGHT_METERS,
        false);

    public static final MirroredSubzone mirroredStageSubzone = new MirroredSubzone(
        "Stage Subzone",
        FieldConstants.BLUE_STAGE_ZONE_SOUTHWEST_CORNER_X_METERS,
        FieldConstants.BLUE_STAGE_ZONE_SOUTHWEST_CORNER_Y_METERS,
        FieldConstants.BLUE_STAGE_ZONE_WIDTH_METERS,
        FieldConstants.BLUE_STAGE_ZONE_HEIGHT_METERS,
        false);

    public static final MirroredSubzone mirroredRobotStartingSubzone = new MirroredSubzone(
        "Robot Starting Subzone",
        FieldConstants.BLUE_ROBOT_STARTING_ZONE_SOUTHWEST_CORNER_X_METERS,
        FieldConstants.BLUE_ROBOT_STARTING_ZONE_SOUTHWEST_CORNER_Y_METERS,
        FieldConstants.BLUE_ROBOT_STARTING_ZONE_WIDTH_METERS,
        FieldConstants.BLUE_ROBOT_STARTING_ZONE_HEIGHT_METERS,
        false);

    public static final MirroredSubzone mirroredSourceSubzone = new MirroredSubzone(
        "Source Subzone",
        FieldConstants.BLUE_SOURCE_ZONE_SOUTHWEST_CORNER_X_METERS,
        FieldConstants.BLUE_SOURCE_ZONE_SOUTHWEST_CORNER_Y_METERS,
        FieldConstants.BLUE_SOURCE_ZONE_WIDTH_METERS,
        FieldConstants.BLUE_SOURCE_ZONE_HEIGHT_METERS,
        false);

    public static final MirroredSubzone mirroredWingSubzone1 = new MirroredSubzone(
        "Wing Subzone 1",
        FieldConstants.BLUE_WING_ZONE_1_SOUTHWEST_CORNER_X_METERS,
        FieldConstants.BLUE_WING_ZONE_1_SOUTHWEST_CORNER_Y_METERS,
        FieldConstants.BLUE_WING_ZONE_1_WIDTH_METERS,
        FieldConstants.BLUE_WING_ZONE_1_HEIGHT_METERS,
        false);

    public static final MirroredSubzone mirroredWingSubzone2 = new MirroredSubzone(
        "Wing Subzone 2",
        FieldConstants.BLUE_WING_ZONE_2_ZONE_SOUTHWEST_CORNER_X_METERS,
        FieldConstants.BLUE_WING_ZONE_2_SOUTHWEST_CORNER_Y_METERS,
        FieldConstants.BLUE_WING_ZONE_2_WIDTH_METERS,
        FieldConstants.BLUE_WING_ZONE_2_HEIGHT_METERS,
        false);

    public static final MirroredSubzone mirroredWingSubzone3 = new MirroredSubzone(
        "Wing Subzone 3",
        FieldConstants.BLUE_WING_ZONE_3_SOUTHWEST_CORNER_X_METERS,
        FieldConstants.BLUE_WING_ZONE_3_SOUTHWEST_CORNER_Y_METERS,
        FieldConstants.BLUE_WING_ZONE_3_WIDTH_METERS,
        FieldConstants.BLUE_WING_ZONE_3_HEIGHT_METERS,
        false);

    public static final MirroredSubzone mirroredWingSubzone4 = new MirroredSubzone(
        "Wing Subzone 4",
        FieldConstants.BLUE_WING_ZONE_4_SOUTHWEST_CORNER_X_METERS,
        FieldConstants.BLUE_WING_ZONE_4_SOUTHWEST_CORNER_Y_METERS,
        FieldConstants.BLUE_WING_ZONE_4_WIDTH_METERS,
        FieldConstants.BLUE_WING_ZONE_4_HEIGHT_METERS,
        false);

    public static final FieldSubzone noneSubzone = new FieldSubzone(
        "None",
        -1,
        -1,
        0,
        0);
    
    public FieldZones() {

        MirroredFieldZone wingMirroredFieldZone = new MirroredFieldZone(FieldMacroZone.WING, "WING", mirroredAmpSubzone);
            wingMirroredFieldZone.addMirroredSubzone(mirroredRobotStartingSubzone);
            wingMirroredFieldZone.addMirroredSubzone(mirroredSourceSubzone);
            wingMirroredFieldZone.addMirroredSubzone(mirroredStageSubzone);
            wingMirroredFieldZone.addMirroredSubzone(mirroredWingSubzone1);
            wingMirroredFieldZone.addMirroredSubzone(mirroredWingSubzone2);
            wingMirroredFieldZone.addMirroredSubzone(mirroredWingSubzone3);
            wingMirroredFieldZone.addMirroredSubzone(mirroredWingSubzone4);

        blueWing = wingMirroredFieldZone.getBlueFieldZone();
        redWing = wingMirroredFieldZone.getRedFieldZone();
        blueWing.generateBoundingBox();
        redWing.generateBoundingBox();
        fieldZones.add(blueWing);
        fieldZones.add(redWing);

        neutralZone = new FieldZone(Alliance.Neither,FieldMacroZone.NEUTRAL, "NEUTRAL ZONE", westNeutralZone);
            neutralZone.addSubzone(eastNeutralZone);
        
        neutralZone.generateBoundingBox();
        fieldZones.add(neutralZone);

        // Create the none zone.
        noneZone = new FieldZone(Alliance.Neither, FieldMacroZone.NONE, "None", noneSubzone);
        noneZone.generateBoundingBox();
        noneSubzone.setFieldZone(noneZone);
        fieldZones.add(noneZone);
    }

    public FieldSubzone getPointFieldZone(Point2D point) {
        // If odometry is working correctly we'll always find a zone, but if it's not, we need a default.
        FieldSubzone robotFieldSubzone = noneSubzone;

        // If odometry is turned off, there is no point spending CPU on testing the bounding boxes.
        if (true) {
            // Check each zone to see if the point is within the overall bounding box.
            // If it is, check the subzones to see if its in one of those bounding boxes.
            // If it is, that's the zone/subzone that the robot is centered in.
            // If it is not, hop back out and continue checking the remaining zones.
            for (int i = 0; i < fieldZones.size(); i++) {
                if (fieldZones.get(i).containsPoint(point)) {
                    robotFieldSubzone = fieldZones.get(i).subzonesContainPoint(point);

                    if (robotFieldSubzone != null) {
                        break;
                    }
                }
            }
        }
        
        if (robotFieldSubzone == null) {
            robotFieldSubzone = FieldZones.noneSubzone;
        }

        return robotFieldSubzone;
    }

    public void output() {
        //System.out.println("Zones:");

        for (FieldZone zone : fieldZones) {
            zone.outputSubzones();
        }

       // System.out.println();
        //System.out.println();
    }

    public FieldZone getNoneZone() {
        return noneZone;
    }

    public ArrayList<FieldZone> getFieldZones() {
        return fieldZones;
    }
}
