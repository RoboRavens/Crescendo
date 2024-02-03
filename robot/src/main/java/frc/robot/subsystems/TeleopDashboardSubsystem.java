package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.util.FieldMeasurements;

public class TeleopDashboardSubsystem extends SubsystemBase {
  
  private StringSubscriber _scoringSub;
  private StringSubscriber _sourceSub;

  private StringEntry _scoringPub;
  private StringEntry _sourcePub;

  private Timer _lockTimer = new Timer();

  public TeleopDashboardSubsystem(){
    var teleopTable = ReactDashSubsystem.ReactDash.getSubTable("Teleop");
    _scoringSub = teleopTable.getStringTopic("dpub/selectedScoreType").subscribe("None");
    
    _scoringPub = teleopTable.getStringTopic("rpub/selectedScoreType").getEntry("None");

    _lockTimer.start();
  }

  @Override
  public void periodic(){
    if (_lockTimer.get() > .5) {
      _scoringPub.set(_scoringSub.get("-1"));
    }
  }

  public void showTab() {
    Robot.REACT_DASH_SUBSYSTEM.SwitchTab(ReactDashSubsystem.TELEOP_TAB_NAME);
  }

  public String getSourceSelection() {
    return _sourcePub.get();
  }

  public String getScoringSelection() {
    return _scoringSub.get();
  }

  public Translation2d getSelectedSourceCoordinates() {
    String selection = getSourceSelection();
    if (Robot.allianceColor == Alliance.Blue) {
        switch (selection) {
            case "Left":
                return FieldMeasurements.BLUE_SOURCES[0];
            case "Middle":
                return FieldMeasurements.BLUE_SOURCES[1];
            case "Right":
                return FieldMeasurements.BLUE_SOURCES[2];
            default:
                return null;
        }
    }
    else if (Robot.allianceColor == Alliance.Red) {
        switch (selection) {
            case "Left":
                return FieldMeasurements.RED_SOURCES[0];
            case "Middle":
                return FieldMeasurements.RED_SOURCES[1];
            case "Right":
                return FieldMeasurements.RED_SOURCES[2];
            default:
                return null;
        }
    }
    return null;
  }

  public Translation2d getSelectedAmpCoordinates() {
    String selection = getScoringSelection();
    if (selection != "Amp") {   
        if (Robot.allianceColor == Alliance.Blue) {
            return FieldMeasurements.BLUE_AMP;
        }
        else if (Robot.allianceColor == Alliance.Red) {
            return FieldMeasurements.RED_AMP;
        }
    }
    return null;
  }
}
