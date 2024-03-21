package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.util.field.FieldConstants;

public class LimelightSubsystem extends SubsystemBase {
  private NetworkTable _baseNetworkTable;
  private String _tableName;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private NetworkTableEntry ts;
  private NetworkTableEntry tv;
  private NetworkTableEntry tl;
  private NetworkTableEntry cl;
  private int camMode = 0;

  private NetworkTableEntry ledMode;
  private NetworkTableEntry _tagId;

  private Timer _bufferedTvTimer = new Timer();
  private boolean _bufferedTv;
  private double _bufferedTx;
  private double _bufferedTy;
  private double _bufferedTa;

  // Defaulting to blue speaker is as good as anything.
  private int _currentTargetTag = 7;

  public LimelightSubsystem(String tableName) {
    _tableName = tableName;
    _baseNetworkTable = NetworkTableInstance.getDefault().getTable(tableName);
    tx = _baseNetworkTable.getEntry("tx");
    ty = _baseNetworkTable.getEntry("ty");
    ta = _baseNetworkTable.getEntry("ta");
    ts = _baseNetworkTable.getEntry("ts");
    tv = _baseNetworkTable.getEntry("tv");
    tl = _baseNetworkTable.getEntry("tl");
    cl = _baseNetworkTable.getEntry("cl");

    ledMode = _baseNetworkTable.getEntry("ledMode");
    _tagId = _baseNetworkTable.getEntry("priorityid");

    _tagId.setNumber(_currentTargetTag);
  }

  public void periodic() {
    // Pose2d robotPose = getPureLimelightRobotPose();
    
    _tagId.setNumber(_currentTargetTag);

    if (this.hasVisionTargetBoolean()) {
      _bufferedTvTimer.reset();
      _bufferedTv = true;
      _bufferedTx = this.getTx();
      _bufferedTx = this.getTy();
      _bufferedTa = this.getTa();
    } else if (_bufferedTvTimer.get() > .25) {
      _bufferedTv = false;
    }



    /*
    if (robotPose != null) {
      SmartDashboard.putNumber(_tableName + "PoseX", robotPose.getX());
      SmartDashboard.putNumber( _tableName + "PoseY", robotPose.getY());
      SmartDashboard.putNumber(_tableName + "Rotation", robotPose.getRotation().getDegrees());
    } else {
      SmartDashboard.putNumber(_tableName + "PoseX", 0);
      SmartDashboard.putNumber(_tableName + "PoseY", 0);
      SmartDashboard.putNumber(_tableName + "Rotation", 0);
    }
    */
  }

  public void setTargetTag(Alliance allianceColor) {
    if (allianceColor == Alliance.Blue) {
      this._currentTargetTag = FieldConstants.BLUE_SPEAKER_CENTER_APRILTAG_ID;
    }
    else if (allianceColor == Alliance.Red) {
      this._currentTargetTag = FieldConstants.RED_SPEAKER_CENTER_APRILTAG_ID;
    }
  }

  public double[] getLimelightBotpose() {
    double[] robotPoseOne = NetworkTableInstance.getDefault().getTable(_tableName).getEntry("botpose_wpiblue")
        .getDoubleArray(new double[6]);

    return robotPoseOne;
  }

  // This gets the robot pose PURELY from the Limelight, instead of using the
  // gyroscope for rotation.
  public Pose2d getPureLimelightRobotPose() {
    double[] botpose = getLimelightBotpose();

    if (botpose.length >= 6) {
      Translation2d translation = new Translation2d(botpose[0], botpose[1]);
      Rotation2d rotation = new Rotation2d(Math.toRadians(botpose[5]));
      Pose2d position = new Pose2d(translation, rotation);
      return position;
    }

    return null;
  }

  // This replaces the Limelight's rotation value with the gyroscope's value.
  public Pose2d getLimelightPoseWithOdometryRotation() {
    double[] botpose = getLimelightBotpose();

    if (botpose.length >= 6) {
      Translation2d translation = new Translation2d(botpose[0], botpose[1]);
      Rotation2d rotation = Robot.DRIVETRAIN_SUBSYSTEM.getOdometryRotation();
      Pose2d position = new Pose2d(translation, rotation);
      return position;
    }

    return null;
  }

  public double hasVisionTarget() {
    return tv.getDouble(0);
  }

  public boolean hasVisionTargetBoolean() {
    return tv.getDouble(0) == 1;
  }

  public boolean hasVisionTargetBuffered() {
    return _bufferedTv;
  }

  public double getCl() {
    return cl.getDouble(0.0);
  }

  public double getTl() {
    return tl.getDouble(0.0);
  }

  public double getTx() {
    return tx.getDouble(0.0);
  }

  public double getBufferedTx() {
    return _bufferedTx;
  }

  public double getTa() {
    return ta.getDouble(0.0);
  }

  public double getBufferedTa() {
    return _bufferedTa;
  }

  public double getTy() {
    return ty.getDouble(0.0);
  }

  public double getBufferedTy() {
    return _bufferedTy;
  }

  public double getTv() {
    return tv.getDouble(0.0);
  }

  public void turnLedOn() {
    ledMode.setNumber(3);
  }

  public void turnLedOff() {
    ledMode.setNumber(1);
  }
}