package frc.robot.subsystems;

import java.util.LinkedHashMap;
import java.util.Map;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.util.AutoMode;

public class AutoChooserSubsystemReact extends SubsystemBase {
  private final Map<String, AutoMode> _autos = new LinkedHashMap<>();

  private AutoMode _redDefault;
  private AutoMode _blueDefault;
  private Alliance _currentAlliance = null;

  private StringArrayPublisher _optionsPub;
  private StringSubscriber _selectedAutoSub;
  private StringPublisher _selectedAutoRobotPub;
  
  private DoubleSubscriber _autoDelaySub;
  private DoublePublisher _autoDelayRobotPub;

  private DoublePublisher _matchTimePub;
  private StringPublisher _alliancePub;

  public AutoChooserSubsystemReact() {
    var autoTable = ReactDashSubsystem.ReactDash.getSubTable("Autonomous");
    _optionsPub = autoTable.getStringArrayTopic("rpub/options").publish();
    _selectedAutoSub = autoTable.getStringTopic("dpub/selectedAuto").subscribe(null);
    _selectedAutoRobotPub = autoTable.getStringTopic("rpub/selectedAuto").publish(PubSubOption.periodic(10));

    _autoDelaySub = autoTable.getDoubleTopic("dpub/autoDelay").subscribe(0);
    _autoDelayRobotPub = autoTable.getDoubleTopic("rpub/autoDelayFromRobot").publish();

    _matchTimePub = autoTable.getDoubleTopic("rpub/matchTime").publish();
    _alliancePub = autoTable.getStringTopic("rpub/alliance").publish();
    
    // BLUE SIDE
    this.addBlueDefault(
      new AutoMode("B1: 6 Note PATH",
      () -> new PathPlannerAuto("6 Note Path"))
    );
    this.addOption(
      new AutoMode("B2: 6 Note AUTO",
      () -> new PathPlannerAuto("6 Note Auto"))
    );
    this.addOption(
      new AutoMode("B3: 3 Note Source Side Center PATH",
      () -> new PathPlannerAuto("Source Side Center Path"))
    );
    this.addOption(
      new AutoMode("B4: 3 Note Source Side Center AUTO BLUE",
      () -> new PathPlannerAuto("Source Side Center Auto Blue"))
    );
    this.addOption(
      new AutoMode("B5: Amp Side Center Then Wing Notes PATH",
      () -> new PathPlannerAuto("Amp Side Center Path"))
    );
    this.addOption(
      new AutoMode("B6: Amp Side Center Then Wing Notes AUTO",
      () -> new PathPlannerAuto("Amp Side Center Auto"))
    );
    this.addOption(
      new AutoMode("B7: Sadville Center Auto",
      () -> new PathPlannerAuto("Sadville Center Auto"))
    );
    this.addOption(
      new AutoMode("B8: 3 Note Source Side Center AUTO RED",
      () -> new PathPlannerAuto("Source Side Center Auto Red"))
    );
    this.addOption(
      new AutoMode("B9: 6 Note Optimized AUTO",
      () -> new PathPlannerAuto("6 Note Optimized Auto"))
    );
    this.addOption(
      new AutoMode("B10: 6 Note Optimized PATH",
      () -> new PathPlannerAuto("6 Note Optimized Path"))
    );
    this.addOption(
      new AutoMode("B11: Sadville Source Side Auto",
      () -> new PathPlannerAuto("Sadville Source Side Auto"))
    );
    this.addOption(
      new AutoMode("B12: Sadville Amp Side Auto",
      () -> new PathPlannerAuto("Sadville Amp Side Auto"))
    );
    this.addOption(
      new AutoMode("B13: Source Side Center Optimized Auto",
      () -> new PathPlannerAuto("Source Side Center Optimized Auto"))
    );
    this.addOption(
      new AutoMode("B14: Amp Side Center Optimized Auto",
      () -> new PathPlannerAuto("Amp Side Center Optimized Auto"))
    );
    this.addOption(
      new AutoMode("B15: 5 Note Center Note First Auto",
      () -> new PathPlannerAuto("5 Note Center Note First Auto"))
    );
    this.addOption(
      new AutoMode("B16: Plowtown Source Side Auto",
      () -> new PathPlannerAuto("Plowtown Source Side Auto"))
    );
    this.addOption(
      new AutoMode("B17: Plowtown Source Side Path",
      () -> new PathPlannerAuto("Plowtown Source Side Path"))
    );

    // RED SIDE
    this.addRedDefault(
      new AutoMode("R1: 6 Note PATH",
      () -> new PathPlannerAuto("6 Note Path"))
    );
    this.addOption(
      new AutoMode("R2: 6 Note AUTO",
      () -> new PathPlannerAuto("6 Note Auto"))
    );
    this.addOption(
      new AutoMode("R3: 3 Note Source Side Center PATH",
      () -> new PathPlannerAuto("Source Side Center Path"))
    );
    this.addOption(
      new AutoMode("R4: 3 Note Source Side Center AUTO BLUE",
      () -> new PathPlannerAuto("Source Side Center Auto Blue"))
    );
    this.addOption(
      new AutoMode("R5: Amp Side Center Then Wing Notes PATH",
      () -> new PathPlannerAuto("Amp Side Center Path"))
    );
    this.addOption(
      new AutoMode("R6: Amp Side Center Then Wing Notes AUTO",
      () -> new PathPlannerAuto("Amp Side Center Auto"))
    );
    this.addOption(
      new AutoMode("R7: Sadville Center Auto",
      () -> new PathPlannerAuto("Sadville Center Auto"))
    );
    this.addOption(
      new AutoMode("R8: 3 Note Source Side Center AUTO RED",
      () -> new PathPlannerAuto("Source Side Center Auto Red"))
    );
    this.addOption(
      new AutoMode("R9: 6 Note Optimized AUTO",
      () -> new PathPlannerAuto("6 Note Optimized Auto"))
    );
    this.addOption(
      new AutoMode("R10: 6 Note Optimized PATH",
      () -> new PathPlannerAuto("6 Note Optimized Path"))
    );
    this.addOption(
      new AutoMode("R11: Sadville Source Side Auto",
      () -> new PathPlannerAuto("Sadville Source Side Auto"))
    );
    this.addOption(
      new AutoMode("R12: Sadville Amp Side Auto",
      () -> new PathPlannerAuto("Sadville Amp Side Auto"))
    );
    this.addOption(
      new AutoMode("R13: Source Side Center Optimized Auto",
      () -> new PathPlannerAuto("Source Side Center Optimized Auto"))
    );
    this.addOption(
      new AutoMode("R14: Amp Side Center Optimized Auto",
      () -> new PathPlannerAuto("Amp Side Center Optimized Auto"))
    );
    this.addOption(
      new AutoMode("R15: 5 Note Center Note First Auto",
      () -> new PathPlannerAuto("5 Note Center Note First Auto"))
    );
    this.addOption(
      new AutoMode("R16: Plowtown Source Side Auto",
      () -> new PathPlannerAuto("Plowtown Source Side Auto"))
    );
    this.addOption(
      new AutoMode("R17: Plowtown Source Side Path",
      () -> new PathPlannerAuto("Plowtown Source Side Path"))
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
    return new WaitCommand(_autoDelaySub.get()).andThen(this.GetAuto().getCommandSupplier().getCommand());
  }

  private AutoMode GetDefaultAuto() {
    return DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Blue ? _blueDefault: _redDefault;
  }

  private void UpdateAlliance(Alliance alliance) {
    var startsWith = alliance == Alliance.Blue ? "B" : "R";
    var keys = _autos.keySet().stream().filter(s -> s.startsWith(startsWith)).toArray(String[]::new);
    _optionsPub.set(keys);
    _currentAlliance = alliance;
  }

  @Override
  public void periodic() {
    var alliance = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue);
    if (_currentAlliance != alliance) {
      this.UpdateAlliance(alliance);
    }

    _selectedAutoRobotPub.set(this.GetAuto().getText());
    _autoDelayRobotPub.set(_autoDelaySub.get());

    _matchTimePub.set(Timer.getMatchTime());
    _alliancePub.set(alliance.name());
  }
}
