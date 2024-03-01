import static org.junit.jupiter.api.Assertions.assertEquals;

import java.awt.geom.Point2D;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.robot.util.field.FieldZones;

public class ZoneTests {

    @Test 
    public void zonetest(){
        FieldZones fieldZones = new FieldZones();
        var zone = fieldZones.getPointFieldZone(new Point2D.Double(15,7));
        assertEquals("Red Robot Starting Subzone", zone.getName());
    }
    
}
