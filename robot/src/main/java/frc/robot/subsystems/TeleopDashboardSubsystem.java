package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.Subscriber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.util.FieldMeasurements;
import frc.util.StateManagement;
import frc.util.StateManagement.ArmUpTargetState;
import frc.util.StateManagement.ClimbPositionTargetState;
import frc.util.StateManagement.IntakeTargetState;
import frc.util.StateManagement.LEDSignalTargetState;
import frc.util.StateManagement.LimelightOverrideState;
import frc.util.StateManagement.ScoringTargetState;
import frc.util.StateManagement.SelectedShotTargetState;
import frc.util.StateManagement.ShooterRevTargetState;
import frc.util.StateManagement.TrapSourceLaneTargetState;

public class TeleopDashboardSubsystem extends SubsystemBase {
  
  private BooleanSubscriber _armUpSub;
  private StringSubscriber _intakeSub;
  private StringSubscriber _sourceLaneSub;
  private StringSubscriber _signalSelectionSub;
  private BooleanSubscriber _startShooterSub;
  private StringSubscriber _shotSelectionSub;
  private BooleanSubscriber _limelightOverrideSub;

  private BooleanEntry _armUpPub;
  private StringEntry _intakePub;
  private StringEntry _sourceLanePub;
  private StringEntry _signalSelectionPub;
  private BooleanEntry _startShooterPub;
  private StringEntry _shotSelectionPub;
  private BooleanEntry _limelightOverridePub;

  private double _armSubLastChange = 0;
  private double _intakeSubLastChange = 0;
  private double _sourceSubLastChange = 0;
  private double _signalSelectionSubLastChange = 0;
  private double _startShooterSubLastChange = 0;
  private double _shotSelectionSubLastChange = 0;
  private double _limelightOverrideSubLastChange = 0;

  private Timer _lockTimer = new Timer();

  public TeleopDashboardSubsystem(){
    var teleopTable = ReactDashSubsystem.ReactDash.getSubTable("Teleop");
    _armUpSub = teleopTable.getBooleanTopic("dpub/armUp").subscribe(false);
    _intakeSub = teleopTable.getStringTopic("dpub/selectedIntakeType").subscribe("ground");
    _sourceLaneSub = teleopTable.getStringTopic("dpub/selectedSourceLane").subscribe("None");
    _signalSelectionSub = teleopTable.getStringTopic("dpub/signalSelection").subscribe("None");
    _startShooterSub = teleopTable.getBooleanTopic("dpub/startShooter").subscribe(false);
    _shotSelectionSub = teleopTable.getStringTopic("dpub/selectedShotType").subscribe("None");
    _limelightOverrideSub = teleopTable.getBooleanTopic("dpub/limelightOverride").subscribe(false);

    _armUpPub = teleopTable.getBooleanTopic("rpub/armUp").getEntry(false);
    _intakePub = teleopTable.getStringTopic("rpub/selectedIntakeType").getEntry("ground");
    _sourceLanePub = teleopTable.getStringTopic("rpub/selectedSourceLane").getEntry("None");
    _signalSelectionPub = teleopTable.getStringTopic("rpub/signalSelection").getEntry("None");
    _startShooterPub = teleopTable.getBooleanTopic("rpub/startShooter").getEntry(false);
    _shotSelectionPub = teleopTable.getStringTopic("rpub/selectedShotType").getEntry("None");
    _limelightOverridePub = teleopTable.getBooleanTopic("rpub/limelightOverride").getEntry(false);

    _lockTimer.start();
  }

  @Override
  public void periodic() {
    if (_lockTimer.get() > .5) {
      _armUpPub.set(Robot.ARM_UP_TARGET_STATE.toString() == "UP");
      _intakePub.set(Robot.INTAKE_TARGET_STATE.toString());
      _sourceLanePub.set(Robot.TRAP_SOURCE_LANE_TARGET_STATE.toString());
      _signalSelectionPub.set(Robot.LED_SIGNAL_TARGET_STATE.toString());
      _startShooterPub.set(Robot.SHOOTER_REV_TARGET_STATE.toString() == "ON");
    }

    // Update the robot target states
    updateStateOnDashboardChange(_intakeSub, "INTAKE_TARGET_STATE", _intakeSubLastChange);
    updateStateOnDashboardChange(_signalSelectionSub, "LED_SIGNAL_TARGET_STATE", _signalSelectionSubLastChange);
    updateStateOnDashboardChange(_sourceLaneSub, "TRAP_SOURCE_LANE_TARGET_STATE", _sourceSubLastChange);
    updateStateOnDashboardChange(_armUpSub, "ARM_UP_TARGET_STATE", _armSubLastChange);
    updateStateOnDashboardChange(_startShooterSub, "SHOOTER_REV_TARGET_STATE", _startShooterSubLastChange);
    updateStateOnDashboardChange(_shotSelectionSub, "SELECTED_SHOT_TARGET_STATE", _shotSelectionSubLastChange);
    updateStateOnDashboardChange(_limelightOverrideSub, "LIMELIGHT_OVERRIDE_STATE", _limelightOverrideSubLastChange);
  }

  /**
   * Updates the robot state when a recent dashboard change is detected
   * We cannot alternatively update the robot state in periodic with calls to subscriber.get() because that will override states set by physical button presses
   * @param subscriber
   * @param state
   * @param storedLastChange
   */
  private void updateStateOnDashboardChange(StringSubscriber subscriber, String state, double storedLastChange) {
    double lastChange = subscriber.getLastChange();
    if (lastChange > storedLastChange) {
      switch (state) {
        case "INTAKE_TARGET_STATE":
          Robot.INTAKE_TARGET_STATE = IntakeTargetState.valueOf(_intakeSub.get("GROUND"));
          _intakeSubLastChange = lastChange;
          break;
        case "LED_SIGNAL_TARGET_STATE":
          Robot.LED_SIGNAL_TARGET_STATE = LEDSignalTargetState.valueOf(_signalSelectionSub.get("NONE"));
          _signalSelectionSubLastChange = lastChange;
          break;
        case "TRAP_SOURCE_LANE_TARGET_STATE":
          Robot.TRAP_SOURCE_LANE_TARGET_STATE = TrapSourceLaneTargetState.valueOf(_sourceLaneSub.get("CENTER"));
          _sourceSubLastChange = lastChange;
          break;
        case "SELECTED_SHOT_TARGET_STATE":
          Robot.SELECTED_SHOT_TARGET_STATE = SelectedShotTargetState.valueOf(_shotSelectionSub.get("SUBWOOFER_SHOT"));
          _shotSelectionSubLastChange = lastChange;
          break;
        default:
          break;
      }
    }
  }

  /**
   * Implementation of updateStateOnDashboardChange() for Boolean Subscribers
   * @param subscriber
   * @param state
   * @param storedLastChange
   */
  private void updateStateOnDashboardChange(BooleanSubscriber subscriber, String state, double storedLastChange) {
    double lastChange = subscriber.getLastChange();
    if (lastChange > storedLastChange) {
      switch (state) {
        case "ARM_UP_TARGET_STATE":
          Robot.ARM_UP_TARGET_STATE = _armUpSub.get(false) 
            ? ArmUpTargetState.UP
            : ArmUpTargetState.FREE;
          _armSubLastChange = lastChange;
          break;
        case "SHOOTER_REV_TARGET_STATE":
          Robot.SHOOTER_REV_TARGET_STATE = _startShooterSub.get(false)
            ? ShooterRevTargetState.ON
            : ShooterRevTargetState.OFF;
          _startShooterSubLastChange = lastChange;
          break;
        case "LIMELIGHT_OVERRIDE_STATE":
          Robot.LIMELIGHT_OVERRIDE_STATE = _limelightOverrideSub.get(false)
            ? LimelightOverrideState.OVERRIDE_ON
            : LimelightOverrideState.OVERRIDE_OFF;
          _limelightOverrideSubLastChange = lastChange;
          break;
        default:
          break;
      }
    }
  }

}
