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
  private StringSubscriber _armSub;
  private StringSubscriber _climbPositionSub;
  private StringSubscriber _intakeSub;

  private StringEntry _scoringPub;
  private StringEntry _armPub;
  private StringEntry _climbPositionPub;
  private StringEntry _intakePub;

  private Timer _lockTimer = new Timer();

  public TeleopDashboardSubsystem(){
    var teleopTable = ReactDashSubsystem.ReactDash.getSubTable("Teleop");
    _scoringSub = teleopTable.getStringTopic("dpub/selectedScoreType").subscribe("None");
    _armSub = teleopTable.getStringTopic("dpub/selectedArmHeight").subscribe("High");
    _climbPositionSub = teleopTable.getStringTopic("dpub/selectedClimbPosition").subscribe("left-center");
    _intakeSub = teleopTable.getStringTopic("dpub/selectedIntakeType").subscribe("ground");

    _scoringPub = teleopTable.getStringTopic("rpub/selectedScoreType").getEntry("None");
    _armPub = teleopTable.getStringTopic("rpub/selectedArmHeight").getEntry("High");
    _climbPositionPub = teleopTable.getStringTopic("rpub/selectedClimbPosition").getEntry("left-center");
    _intakePub = teleopTable.getStringTopic("rpub/selectedIntakeType").getEntry("ground");

    _lockTimer.start();
  }

  @Override
  public void periodic(){
    if (_lockTimer.get() > .5) {
      _scoringPub.set(_scoringSub.get("None"));
      _armPub.set(_armSub.get("None"));
      _climbPositionPub.set(_climbPositionSub.get("None"));
      _intakePub.set(_intakeSub.get("None"));
    }
  }

  public void showTab() {
    Robot.REACT_DASH_SUBSYSTEM.SwitchTab(ReactDashSubsystem.TELEOP_TAB_NAME);
  }

  public String getScoringSelection() {
    return _scoringSub.get();
  }

  public String getArmHeightSelection() {
    return _armSub.get();
  }

  public String getClimbPositionSelection() {
    return _climbPositionSub.get();
  }

  public Translation2d getSelectedClimbPositionCoordinates() {
    // TODO: Implement this method
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
