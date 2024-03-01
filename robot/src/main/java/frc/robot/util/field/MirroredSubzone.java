package frc.robot.util.field;

public class MirroredSubzone {
    private FieldSubzone blueSubzone;
    private FieldSubzone redSubzone;

    public MirroredSubzone(String name, double southwestCornerX, double southwestCornerY, double width, double height, boolean flipZones) {
        if (flipZones) {
            blueSubzone = new FieldSubzone("Blue " + name, convertToRed(southwestCornerX + width), southwestCornerY, width, height);
            redSubzone = new FieldSubzone("Red " + name, southwestCornerX, southwestCornerY, width, height);
        }
        else {        
            blueSubzone = new FieldSubzone("Blue " + name, southwestCornerX, southwestCornerY, width, height);
            redSubzone = new FieldSubzone("Red " + name, convertToRed(southwestCornerX + width), southwestCornerY, width, height);
        }
    }

    public static double convertToRed(double blueAllianceWidth) {
        return FieldConstants.FIELD_WIDTH_METERS - blueAllianceWidth;
    }

    public FieldSubzone getBlueSubzone() {
        return blueSubzone;
    }

    public FieldSubzone getRedSubzone() {
        return redSubzone;
    }
}
