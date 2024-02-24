package frc.robot.util.field;

import java.util.ArrayList;

import java.awt.geom.Point2D;


public class FieldZones {
    public enum FieldMacroZone {
        NONE,
        WING,
        STAGE,
        AMP,
        SOURCE,
        STARTING,
        NEUTRAL
    }

    private ArrayList<FieldZone> fieldZones = new ArrayList<FieldZone>();
    private FieldZone blueWing;
    
    
    // We'll include a none state in case odometry ever breaks by enough that no zone is returned.
    private FieldZone noneZone;

    // Field Zones for Rapid React
    public static final FieldSubzone blueInsideWing = new FieldSubzone(
        "Blue Inside Wing",
        0,
        0,
        5.84,
        8.23);

    public static final FieldSubzone ampSubzone = new FieldSubzone("Blue Amp Subzone", 0,0,0,0);

    
    public static final FieldSubzone noneSubzone = new FieldSubzone(
        "None",
        -1,
        -1,
        0,
        0);

    
    
    public FieldZones() {

        blueWing = new FieldZone(Alliance.Blue, FieldMacroZone.NONE, "blue wing", blueInsideWing);
        blueWing.generateBoundingBox();
        fieldZones.add(blueWing);

        // Communities.
        
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
        // ArrayList<FieldZone> rapidReactFieldZones = new ArrayList<>();
        // rapidReactFieldZones.add(new FieldZone(Alliance.Blue, FieldMacroZone.NONE, "Blue Inside Safe Zone", blueInsideSafeZone));
        // rapidReactFieldZones.add(new FieldZone(Alliance.Blue, FieldMacroZone.NONE, "Blue Outside Safe Zone", blueOutsideSafeZone));
        // rapidReactFieldZones.add(new FieldZone(Alliance.Red, FieldMacroZone.NONE, "Red Inside Safe Zone", redInsideSafeZone));
        // rapidReactFieldZones.add(new FieldZone(Alliance.Red, FieldMacroZone.NONE, "Red Outside Safe Zone", redOutsideSafeZone));
        // return rapidReactFieldZones;
    }
}
