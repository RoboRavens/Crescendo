package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightPickupSubsystem extends SubsystemBase {
  private NetworkTable _baseNetworkTable;
  private NetworkTableEntry tx;
  private NetworkTableEntry tv;
  private NetworkTableEntry ta;

  private Timer _bufferedTvTimer = new Timer();
  private boolean _bufferedTv;
  private double _bufferedTx;
  private double _bufferedTa;

  public LimelightPickupSubsystem(String tableName) {
    _baseNetworkTable = NetworkTableInstance.getDefault().getTable(tableName);
    tx = _baseNetworkTable.getEntry("tx");
    tv = _baseNetworkTable.getEntry("tv");
    ta = _baseNetworkTable.getEntry("ta");
    _bufferedTvTimer.start();
  }

  public void periodic() {
    if (this.hasVisionTarget()) {
      _bufferedTvTimer.reset();
      _bufferedTv = true;
      _bufferedTx = this.getTx();
      _bufferedTa = this.getTa();
    } else if (_bufferedTvTimer.get() > .25) {
      _bufferedTv = false;
    }
  }

  private boolean hasVisionTarget() {
    return tv.getDouble(0) == 1;
  }

  public boolean hasVisionTargetBuffered() {
    return _bufferedTv;
  }

  private double getTx() {
    return tx.getDouble(0.0);
  }

  public double getBufferedTx() {
    return _bufferedTx;
  }

  private double getTa() {
    return ta.getDouble(0.0);
  }

  public double getBufferedTa() {
    return _bufferedTa;
  }
}