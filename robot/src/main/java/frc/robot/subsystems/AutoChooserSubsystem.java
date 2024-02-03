package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.commands.auto.*;
import frc.util.AutoMode;

public class AutoChooserSubsystem extends SubsystemBase {
  private SendableChooser<AutoMode> _chooser = new SendableChooser<AutoMode>();
  private String _tabName = "Autonomous";
  private ShuffleboardTab _tab = Shuffleboard.getTab(_tabName);
  private GenericEntry _chosenAutoText;
  private GenericEntry _scoringTabGo;
  private GenericEntry _gameTimeEntry;

  public void BuildAutoChooser(Alliance alliance) {
    switch(alliance){
      case Blue:
        this.addDefaultOption(
          new AutoMode("B1: Test Drive Straight",
          () -> TestAutoCommand.getAutoMode())
        );
        // this.addOption(
        //   new AutoMode("B2: LZ-side 2 cone + balance",
        //   () -> TwoConeAndBalanceAutoCommand.getAutoMode(AutoEnums.AutoAlliance.Blue, AutoEnums.AutoSide.LZ))
        // );

        _tab
          .add("blue", true)
          .withWidget(BuiltInWidgets.kBooleanBox)
          .withProperties(Map.of("Color when true", "#0000FF"))
          .withPosition(0, 0)
          .withSize(1, 1)
          .getEntry();

        _tab.add("blue auto", _chooser)
          .withPosition(1, 0)
          .withSize(4, 1);
        break;
      case Red:
        this.addDefaultOption(
          new AutoMode("R1: Test Drive Straight",
          () -> TestAutoCommand.getAutoMode())
        );
        // this.addOption(
        //   new AutoMode("R2: LZ-side 2 cone + balance",
        //   () -> TwoConeAndBalanceAutoCommand.getAutoMode(AutoEnums.AutoAlliance.Red, AutoEnums.AutoSide.LZ))
        // );

        _tab
          .add("red", true)
          .withWidget(BuiltInWidgets.kBooleanBox)
          .withProperties(Map.of("Color when true", "#FF0000"))
          .withPosition(0, 1)
          .withSize(1, 1)
          .getEntry();

        _tab.add("red auto", _chooser)
          .withPosition(1, 1)
          .withSize(4, 1);
        break;
      default:
      this.addDefaultOption(new AutoMode("Invalid Alliance"));
    }

    _chosenAutoText = _tab.add("chosen auto", "")
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(1, 2)
      .withSize(4, 1)
      .getEntry();

    _scoringTabGo = _tab
      .add("scoring tab", false)
      .withWidget(BuiltInWidgets.kToggleButton)
      .withPosition(5, 2)
      .withSize(1, 1)
      .getEntry();

    _gameTimeEntry = _tab
      .add("Game Time", 0.0)
      .withWidget(BuiltInWidgets.kDial)
      .withProperties(Map.of("min", 0, "max", 15))
      .withPosition(5, 0)
      .withSize(2, 2)
      .getEntry();
  }

  @Override
  public void periodic() {
    var chosen = _chooser.getSelected();
    _chosenAutoText.setString(chosen.getText());
    _gameTimeEntry.setDouble(Timer.getMatchTime());

    // if(_scoringTabGo.getBoolean(false)) {
    //   Robot.TABLET_SCORING_SUBSYSTEM.ShowTab();
    //   _scoringTabGo.setBoolean(false);
    // }
  }

  public void ShowTab() {
    Shuffleboard.selectTab(_tabName);
  }

  public Command GetAutoCommand() {
    var chosen = _chooser.getSelected();
    return chosen.getCommandSupplier().getCommand();
  }

  public void addDefaultOption(AutoMode auto) {
    _chooser.setDefaultOption(auto.getText(), auto);
  }

  private void addOption(AutoMode auto) {
    _chooser.addOption(auto.getText(), auto);
  }
}
