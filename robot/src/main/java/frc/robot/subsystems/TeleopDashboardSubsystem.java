package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.util.FieldMeasurements;
import frc.util.StateManagement.ArmUpTargetState;
import frc.util.StateManagement.ClimbPositionTargetState;
import frc.util.StateManagement.IntakeTargetState;
import frc.util.StateManagement.LEDSignalTargetState;
import frc.util.StateManagement.ScoringTargetState;
import frc.util.StateManagement.ShooterRevTargetState;
import frc.util.StateManagement.TrapSourceLaneTargetState;

public class TeleopDashboardSubsystem extends SubsystemBase {
  
  private StringSubscriber _scoringSub;
  private BooleanSubscriber _armUpSub;
  private StringSubscriber _climbPositionSub;
  private StringSubscriber _intakeSub;
  private StringSubscriber _sourceLaneSub;
  private StringSubscriber _signalSelectionSub;
  private BooleanSubscriber _startShooterSub;

  private StringEntry _scoringPub;
  private BooleanEntry _armUpPub;
  private StringEntry _climbPositionPub;
  private StringEntry _intakePub;
  private StringEntry _sourceLanePub;
  private StringEntry _signalSelectionPub;
  private BooleanEntry _startShooterPub;

  private Timer _lockTimer = new Timer();

  public TeleopDashboardSubsystem(){
    var teleopTable = ReactDashSubsystem.ReactDash.getSubTable("Teleop");
    _scoringSub = teleopTable.getStringTopic("dpub/selectedScoreType").subscribe("None");
    _armUpSub = teleopTable.getBooleanTopic("dpub/armUp").subscribe(false);
    _climbPositionSub = teleopTable.getStringTopic("dpub/selectedClimbPosition").subscribe("left-center");
    _intakeSub = teleopTable.getStringTopic("dpub/selectedIntakeType").subscribe("ground");
    _sourceLaneSub = teleopTable.getStringTopic("dpub/selectedSourceLane").subscribe("None");
    _signalSelectionSub = teleopTable.getStringTopic("dpub/signalSelection").subscribe("None");
    _startShooterSub = teleopTable.getBooleanTopic("dpub/startShooter").subscribe(false);

    _scoringPub = teleopTable.getStringTopic("rpub/selectedScoreType").getEntry("None");
    _armUpPub = teleopTable.getBooleanTopic("rpub/armUp").getEntry(false);
    _climbPositionPub = teleopTable.getStringTopic("rpub/selectedClimbPosition").getEntry("left-center");
    _intakePub = teleopTable.getStringTopic("rpub/selectedIntakeType").getEntry("ground");
    _sourceLanePub = teleopTable.getStringTopic("rpub/selectedSourceLane").getEntry("None");
    _signalSelectionPub = teleopTable.getStringTopic("rpub/signalSelection").getEntry("None");
    _startShooterPub = teleopTable.getBooleanTopic("rpub/startShooter").getEntry(false);

    _lockTimer.start();
  }

  @Override
  public void periodic(){
    if (_lockTimer.get() > .5) {
      _scoringPub.set(Robot.SCORING_TARGET_STATE.toString());
      System.out.println(_scoringPub.get());
    }

    // Can we find out when the last change was from the dashboard and only update the value if there was a recent change?
    // May prevent states from being immediately set back to the sub value after they are set by the button panel
    // Update the robot target states
    Robot.SCORING_TARGET_STATE = ScoringTargetState.valueOf(_scoringSub.get("SPEAKER"));
    Robot.INTAKE_TARGET_STATE = IntakeTargetState.valueOf(_intakeSub.get("GROUND"));
    Robot.LED_SIGNAL_TARGET_STATE = LEDSignalTargetState.valueOf(_signalSelectionSub.get("NONE"));
    Robot.TRAP_SOURCE_LANE_TARGET_STATE = TrapSourceLaneTargetState.valueOf(_sourceLaneSub.get("CENTER"));
    Robot.CLIMB_POSITION_TARGET_STATE = ClimbPositionTargetState.valueOf(_climbPositionSub.get("LEFT_CLOSE"));
    Robot.ARM_UP_TARGET_STATE = _armUpSub.get(false) 
      ? ArmUpTargetState.UP
      : ArmUpTargetState.FREE;
    Robot.SHOOTER_REV_TARGET_STATE = _startShooterSub.get(false)
      ? ShooterRevTargetState.ON
      : ShooterRevTargetState.OFF;
  }

}
