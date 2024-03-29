package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.util.Constants.Constants;

public class PoseEstimatorSubsystem extends SubsystemBase {
    public final Field2d _field = new Field2d();
    private final Field2d _field2dLeft = new Field2d();
    private final Field2d _field2dRight = new Field2d();
    private final Field2d _field2dBack = new Field2d();
    private double _timeStamp = Timer.getFPGATimestamp() - (Robot.LIMELIGHT_BACK.getTl() / 1000)
        - (Robot.LIMELIGHT_BACK.getCl() / 1000);
    private double _timeStamp2 = Timer.getFPGATimestamp() - (Robot.LIMELIGHT_LEFT.getTl() / 1000)
        - (Robot.LIMELIGHT_LEFT.getCl() / 1000);
    private double _timeStamp3 = Timer.getFPGATimestamp() - (Robot.LIMELIGHT_RIGHT.getTl() / 1000)
        - (Robot.LIMELIGHT_RIGHT.getCl() / 1000);
    private Matrix<N3, N1> _stateStdDevs = VecBuilder.fill(Constants.STATE_STANDARD_DEVIATION,
        Constants.STATE_STANDARD_DEVIATION, Constants.STATE_STANDARD_DEVIATION);
    private Matrix<N3, N1> _visionStdDevs = VecBuilder.fill(Constants.STARTING_VISION_STANDARD_DEVIATION,
        Constants.STARTING_VISION_STANDARD_DEVIATION, Constants.STARTING_VISION_STANDARD_DEVIATION);

    public PoseEstimatorSubsystem() {
        SmartDashboard.putData("PoseEstimator Field", _field);
        SmartDashboard.putData("Limelight Back Field2d", _field2dBack);
        SmartDashboard.putData("Limelight Left Field2d", _field2dLeft);
        SmartDashboard.putData("Limelight Right Field2d", _field2dRight);
    }

    @Override
    public void periodic() {
        updateVisionMeasurementTimestamp();
        updateVisionMeasurmentTimestampSecondLimelight();
        updateVisionMeasurmentTimestampThirdLimelight();
        double ta = Robot.LIMELIGHT_BACK.getTa();
        double ta2 = Robot.LIMELIGHT_LEFT.getTa();
        double ta3 = Robot.LIMELIGHT_RIGHT.getTa();

        //SmartDashboard.putNumber("ta", ta);

  //      SmartDashboard.putNumber("ta2", ta2);

        Pose2d firstLimelightPose = Robot.LIMELIGHT_BACK.getLimelightPoseWithOdometryRotation();
        Pose2d secondLimelightPose = Robot.LIMELIGHT_LEFT.getLimelightPoseWithOdometryRotation();
        Pose2d thirdLimelightPose = Robot.LIMELIGHT_RIGHT.getLimelightPoseWithOdometryRotation();
        //Pose2d robotPose = Robot.POSE_ESTIMATOR_SUBSYSTEM.getCurrentPose();
        //Robot.LIMELIGHT_SUBSYSTEM_ONE.getPureLimelightRobotPose();
        //Robot.DRIVE_TRAIN_SUBSYSTEM.getPose();
        //Robot.DRIVE_TRAIN_SUBSYSTEM.getGyroscopeRotation();
        //Robot.DRIVE_TRAIN_SUBSYSTEM.getSwerveModulePositions();

        boolean hasVisionTarget = Robot.LIMELIGHT_BACK.hasVisionTarget() == 1;
        boolean hasVisionTarget2 = Robot.LIMELIGHT_LEFT.hasVisionTarget() == 1;
        boolean hasVisionTarget3 = Robot.LIMELIGHT_RIGHT.hasVisionTarget() == 1;
        //boolean firstLimelightIsWithinXDistance = Math.abs(robotPose.getX() - firstLimelightPose.getX()) < 2;
        //boolean firstLimelightIsWithinYDistance = Math.abs(robotPose.getY() - firstLimelightPose.getY()) < 2;
        //boolean secondLimelightIsWithinXDistance = Math.abs(robotPose.getX() - secondLimelightPose.getX()) < 2;
        //boolean secondLimelightIsWithinYDistance = Math.abs(robotPose.getY() - secondLimelightPose.getY()) < 2;
        boolean targetAreaIsSufficient = ta >= 0.4;
        boolean targetAreaIsSufficient2 = ta2 >= 0.4;
        boolean targetAreaIsSufficient3 = ta3 >= 0.4;
        boolean fiducialIdIsCorrect3 = LimelightHelpers.getFiducialID("limelight-three") <= 8;
        boolean fiducialIdIsCorrect2 = LimelightHelpers.getFiducialID("limelight-two") <= 8;
        boolean fiducialIdIsCorrect = LimelightHelpers.getFiducialID("limelight") <= 8;

        if (hasVisionTarget /*&& firstLimelightIsWithinXDistance && firstLimelightIsWithinYDistance*/
                && targetAreaIsSufficient && fiducialIdIsCorrect) {
            //SmartDashboard.putBoolean("limelight has target", true);
            var vsStdDvs = PoseEstimatorSubsystem.GetVisionStdDevs(ta);
            m_poseEstimator.addVisionMeasurement(firstLimelightPose, _timeStamp, vsStdDvs);
        } else {
            //SmartDashboard.putBoolean("limelight has target", false);
        }

        if (hasVisionTarget2 /*&& secondLimelightIsWithinXDistance && secondLimelightIsWithinYDistance*/
                && targetAreaIsSufficient2 && fiducialIdIsCorrect2) {
            //SmartDashboard.putBoolean("limelight2 has target", true);
            var vsStdDvs = PoseEstimatorSubsystem.GetVisionStdDevs(ta2);
            m_poseEstimator.addVisionMeasurement(secondLimelightPose, _timeStamp2, vsStdDvs);
        } else {
            //SmartDashboard.putBoolean("limelight2 has target", false);
        }

        if (hasVisionTarget3 /*&& secondLimelightIsWithinXDistance && secondLimelightIsWithinYDistance*/
                && targetAreaIsSufficient3 && fiducialIdIsCorrect3) {
            //SmartDashboard.putBoolean("limelight2 has target", true);
            var vsStdDvs = PoseEstimatorSubsystem.GetVisionStdDevs(ta3);
            m_poseEstimator.addVisionMeasurement(thirdLimelightPose, _timeStamp3, vsStdDvs);
        } else {
            //SmartDashboard.putBoolean("limelight2 has target", false);
        }

        updateOdometry();
        // var pose = getCurrentPose();
        // _field.setRobotPose(pose);
        // SmartDashboard.putNumber("PoseEstimator X", pose.getX());
        // SmartDashboard.putNumber("PoseEstimator Y", pose.getY());
        // SmartDashboard.putNumber("PoseEstimator Rotation (Degrees)", pose.getRotation().getDegrees());
        // SmartDashboard.putString("Limelight 1 Pose", Robot.LIMELIGHT_PICKUP.getPureLimelightRobotPose().toString());
        // SmartDashboard.putString("Limelight 2 Pose", Robot.LIMELIGHT_LEFT.getPureLimelightRobotPose().toString());
        // SmartDashboard.putString("Limelight 3 Pose", Robot.LIMELIGHT_RIGHT.getPureLimelightRobotPose().toString());
        _field.setRobotPose(getCurrentPose());
        _field2dBack.setRobotPose(Robot.LIMELIGHT_BACK.getPureLimelightRobotPose());
        _field2dLeft.setRobotPose(Robot.LIMELIGHT_LEFT.getPureLimelightRobotPose());
        _field2dRight.setRobotPose(Robot.LIMELIGHT_RIGHT.getPureLimelightRobotPose());
    }

    private static Matrix<N3, N1> GetVisionStdDevs(double ta) {
        // Scale the confidence of the vision estimate by how much ApilTag we see.
        ta = Math.min(ta, 1);
        double inverseArea = 1 - ta;
        double inverseCubed = Math.pow(inverseArea, 3);
        double clampedCube = Math.max(inverseCubed, Constants.MINIMUM_VISION_STANDARD_DEVIATION);

//        SmartDashboard.putNumber("VisionStdDev", clampedCube);

        Matrix<N3, N1> visionStdDevs = VecBuilder.fill(clampedCube, clampedCube, clampedCube);

        return visionStdDevs;
    }

    public void updateVisionMeasurementTimestamp() {
        _timeStamp = Timer.getFPGATimestamp() - (Robot.LIMELIGHT_BACK.getTl() / 1000)
            - (Robot.LIMELIGHT_BACK.getCl() / 1000);
    }

    public void updateVisionMeasurmentTimestampSecondLimelight() {
        _timeStamp2 = Timer.getFPGATimestamp() - (Robot.LIMELIGHT_LEFT.getTl() / 1000)
            - (Robot.LIMELIGHT_LEFT.getCl() / 1000);
    }

    public void updateVisionMeasurmentTimestampThirdLimelight() {
        _timeStamp3 = Timer.getFPGATimestamp() - (Robot.LIMELIGHT_RIGHT.getTl() / 1000)
            - (Robot.LIMELIGHT_RIGHT.getCl() / 1000);
    }

    private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
        Robot.DRIVETRAIN_SUBSYSTEM.m_kinematics,
        Robot.DRIVETRAIN_SUBSYSTEM.getGyroscopeRotation(),
        Robot.DRIVETRAIN_SUBSYSTEM.getSwerveModulePositions(),
        new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
        _stateStdDevs,
        _visionStdDevs);

    public void resetOdometryPoseToLimelight() {
        var gyroBasedPose = Robot.LIMELIGHT_BACK.getLimelightPoseWithOdometryRotation();
       // System.out.println("resetOdometryPoseToLimelight: " + gyroBasedPose.getX() + " - " + gyroBasedPose.getY()
       //         + " - " + gyroBasedPose.getRotation().getDegrees());
        m_poseEstimator.resetPosition(
            Robot.DRIVETRAIN_SUBSYSTEM.getGyroscopeRotation(),
            Robot.DRIVETRAIN_SUBSYSTEM.getSwerveModulePositions(),
            gyroBasedPose);
    }

    public void resetOdometryPoseFromSecondLimelight(Rotation2d rotation2d, SwerveModulePosition[] modulePositions,
            Pose2d poseMeters) {
        // Reset state estimate and error covariance
        m_poseEstimator.resetPosition(Robot.DRIVETRAIN_SUBSYSTEM.getGyroscopeRotation(),
            Robot.DRIVETRAIN_SUBSYSTEM.getSwerveModulePositions(),
            Robot.LIMELIGHT_LEFT.getLimelightPoseWithOdometryRotation());
    }

    public void resetOdometryPoseFromThirdLimelight(Rotation2d rotation2d, SwerveModulePosition[] modulePositions,
            Pose2d poseMeters) {
        // Reset state estimate and error covariance
        m_poseEstimator.resetPosition(Robot.DRIVETRAIN_SUBSYSTEM.getGyroscopeRotation(),
            Robot.DRIVETRAIN_SUBSYSTEM.getSwerveModulePositions(),
            Robot.LIMELIGHT_RIGHT.getLimelightPoseWithOdometryRotation());
    }

    // public void addVisionMeasurment(Pose2d robotPose, double timestampSeconds) {
    // m_poseEstimator.addVisionMeasurement(robotPose, timestampSeconds,
    // stateStdDevs);

    // /*
    // m_poseEstimator.addVisionMeasurement(
    // // m_poseEstimator.getEstimatedPosition(),
    // Robot.LIMELIGHT_SUBSYSTEM.getPureLimelightRobotPose(),
    // timeStamp);
    // */
    // }

    public void updateOdometry() {
        m_poseEstimator.update(
            Robot.DRIVETRAIN_SUBSYSTEM.getGyroscopeRotation(),
            Robot.DRIVETRAIN_SUBSYSTEM.getSwerveModulePositions());
    }

    public void resetPosition(Pose2d pose) {
        m_poseEstimator.resetPosition(
            Robot.DRIVETRAIN_SUBSYSTEM.getGyroscopeRotation(),
            Robot.DRIVETRAIN_SUBSYSTEM.getSwerveModulePositions(),
            pose);
    }

    public void zeroGyroscope() {
        var pose = m_poseEstimator.getEstimatedPosition();
        m_poseEstimator.resetPosition(
            Robot.DRIVETRAIN_SUBSYSTEM.getGyroscopeRotation(),
            Robot.DRIVETRAIN_SUBSYSTEM.getSwerveModulePositions(),
            new Pose2d(pose.getTranslation(), new Rotation2d()));
    }

    public Pose2d getCurrentPose() {
        return m_poseEstimator.getEstimatedPosition();
    }
}