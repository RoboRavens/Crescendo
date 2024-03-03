package frc.robot.subsystems;

import java.util.LinkedHashMap;
import java.util.Map;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.commands.auto.*;
import frc.util.AutoMode;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class AutoChooserSubsystemReact extends SubsystemBase {
  private final Map<String, AutoMode> _autos = new LinkedHashMap<>();

  private AutoMode _redDefault;
  private AutoMode _blueDefault;
  private Alliance _currentAlliance = null;

  private StringArrayPublisher _optionsPub;
  private StringSubscriber _selectedAutoSub;
  private StringPublisher _selectedAutoRobotPub;

  private DoublePublisher _matchTimePub;
  private StringPublisher _alliancePub;

  public AutoChooserSubsystemReact() {
    var autoTable = ReactDashSubsystem.ReactDash.getSubTable("Autonomous");
    _optionsPub = autoTable.getStringArrayTopic("rpub/options").publish();
    _selectedAutoSub = autoTable.getStringTopic("dpub/selectedAuto").subscribe(null);
    _selectedAutoRobotPub = autoTable.getStringTopic("rpub/selectedAuto").publish(PubSubOption.periodic(10));

    _matchTimePub = autoTable.getDoubleTopic("rpub/matchTime").publish();
    _alliancePub = autoTable.getStringTopic("rpub/alliance").publish();
    
    // BLUE SIDE
    this.addBlueDefault(
      new AutoMode("B1: Six Note Path",
      () -> new PathPlannerAuto("SixNoteBlue"))
    );
    this.addOption(
      new AutoMode("B2: North Midfield Path",
      () -> new PathPlannerAuto("NorthMidfieldBlue"))
    );
    this.addOption(
      new AutoMode("B3: South Midfield Path",
      () -> new PathPlannerAuto("SouthMidfieldBlue"))
    );
    this.addOption(
      new AutoMode("B4: Six Note Auto",
      () -> SixNoteAutoCommand.getAutoMode())
    );
    this.addOption(
      new AutoMode("B5: North Midfield Auto",
      () -> NorthMidfieldAutoCommand.getAutoMode())
    );
    this.addOption(
      new AutoMode("B6: South Midfield Auto",
      () -> SouthMidfieldAutoCommand.getAutoMode())
    );
    this.addOption(
      new AutoMode("B7: 2",
      () -> new PathPlannerAuto("SixNoteTest2"))
    );

    // RED SIDE
    this.addRedDefault(
      new AutoMode("R1: Six Note Path",
      () -> new PathPlannerAuto("SixNoteBlue"))
    );
    this.addOption(
      new AutoMode("R2: North Midfield Path",
      () -> new PathPlannerAuto("NorthMidfieldBlue"))
    );
    this.addOption(
      new AutoMode("R3: South Midfield Path",
      () -> new PathPlannerAuto("SouthMidfieldBlue"))
    );
    this.addOption(
      new AutoMode("R4: Six Note Auto",
      () -> SixNoteAutoCommand.getAutoMode())
    );
    this.addOption(
      new AutoMode("R5: North Midfield Auto",
      () -> NorthMidfieldAutoCommand.getAutoMode())
    );
    this.addOption(
      new AutoMode("R6: South Midfield Auto",
      () -> new PathPlannerAuto("SixNoteTest2"))
    );
  }

  private void addOption(AutoMode auto) {
    _autos.put(auto.getText(), auto);
  }

  private void addBlueDefault(AutoMode auto) {
    _blueDefault = auto;
    _autos.put(auto.getText(), auto);
  }

  private void addRedDefault(AutoMode auto) {
    _redDefault = auto;
    _autos.put(auto.getText(), auto);
  }

  private AutoMode GetAuto() {
    String selectedAuto = _selectedAutoSub.get(null);
    if (selectedAuto == null) {
      return this.GetDefaultAuto();
    }

    var chosen = _autos.getOrDefault(selectedAuto, null);
    if (chosen == null) {
      return this.GetDefaultAuto();
    }

    return chosen;
  }

  public void BuildAutoChooser(Alliance allianceColor) {
    // does nothing, here for backward compatibility
  }

  public void ShowTab() {
    Robot.REACT_DASH_SUBSYSTEM.SwitchTab(ReactDashSubsystem.AUTO_TAB_NAME);
  }

  public Command GetAutoCommand() {
    return this.GetAuto().getCommandSupplier().getCommand();
  }

  private AutoMode GetDefaultAuto() {
    return DriverStation.getAlliance().get() == Alliance.Blue ? _blueDefault: _redDefault;
  }

  private void UpdateAlliance(Alliance alliance) {
    var startsWith = alliance == Alliance.Blue ? "B" : "R";
    var keys = _autos.keySet().stream().filter(s -> s.startsWith(startsWith)).toArray(String[]::new);
    _optionsPub.set(keys);
    _currentAlliance = alliance;
  }

  @Override
  public void periodic() {
    var alliance = DriverStation.getAlliance().get();
    if (_currentAlliance != alliance) {
      this.UpdateAlliance(alliance);
    }

    _selectedAutoRobotPub.set(this.GetAuto().getText());

    _matchTimePub.set(Timer.getMatchTime());
    _alliancePub.set(DriverStation.getAlliance().get().name());
  }
}
