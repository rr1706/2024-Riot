package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.utilities.MathUtils;

public class PoseEstimator extends SubsystemBase {
    private final SwerveDrivePoseEstimator m_poseEstimator;
    //private final SwerveDrivePoseEstimator m_noteEstimator;
    private final Drivetrain m_drivetrain;
    private final Field2d m_field = new Field2d();
    private boolean m_auto = true;
    //private boolean m_trackNote = false;
    //private int m_noteID = 0;
    //private InterpolatingDoubleTreeMap m_noteDistance = new InterpolatingDoubleTreeMap();

    public PoseEstimator(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kSwerveKinematics,
                drivetrain.getGyro(),
                drivetrain.getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.229, 0.229, 0.229),
                VecBuilder.fill(10, 10, 200));
        // m_noteEstimator = new SwerveDrivePoseEstimator(DriveConstants.kSwerveKinematics,
        //         drivetrain.getGyro(),
        //         drivetrain.getModulePositions(),
        //         new Pose2d(),
        //         VecBuilder.fill(0.229, 0.229, 0.229),
        //         VecBuilder.fill(10, 10, 200));
        // m_noteDistance = MathUtils.pointsToTreeMap(VisionConstants.kNoteDistance);

        SmartDashboard.putData("Field", m_field);
    }

    @Override
    public void periodic() {
        updatePoseEstimator(false);
        updateShuffleboard();
    }

    private void updatePoseEstimator(boolean force) {
        SmartDashboard.putBoolean("Auto Pose", m_auto);

        m_poseEstimator.updateWithTime(Timer.getFPGATimestamp(), m_drivetrain.getGyro(),
                m_drivetrain.getModulePositions());
        // m_noteEstimator.updateWithTime(Timer.getFPGATimestamp(), m_drivetrain.getGyro(),
        //         m_drivetrain.getModulePositions());
        updateWithVision("limelight-front");
        updateWithVision("limelight-back");

        // if (m_trackNote && LimelightHelpers.getTV("limelight-note")) {
        //     //updateWithNote("limelight-note", m_noteID);
        // }

    }

    private void updateWithVision(String limelightName) {
        double ta = LimelightHelpers.getTA(limelightName);
        PoseEstimate limelightBotPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        int validTagCount = limelightBotPose.tagCount;
        boolean slowRotate = m_drivetrain.getChassisSpeed().omegaRadiansPerSecond <= 4 * Math.PI;

        if ((validTagCount >= 2.0 && ta >= 0.030) && slowRotate) {
            if (m_auto) {
                double antiTrust = 4*(-150.0 * ta + 10.0);
                if (antiTrust <= 8.0) {
                    antiTrust = 8.0;
                }                m_poseEstimator.addVisionMeasurement(limelightBotPose.pose, limelightBotPose.timestampSeconds,
                        VecBuilder.fill(antiTrust, antiTrust, antiTrust));
            } else {
                double antiTrust = -150.0 * ta + 10.0;
                if (antiTrust <= 2.0) {
                    antiTrust = 2.0;
                }
                m_poseEstimator.addVisionMeasurement(limelightBotPose.pose, limelightBotPose.timestampSeconds,
                        VecBuilder.fill(antiTrust, antiTrust, antiTrust));

            }
        } else if ((validTagCount == 1 && ta >= 0.060) && !m_auto && slowRotate) {
            if (m_auto) {
                m_poseEstimator.addVisionMeasurement(limelightBotPose.pose, limelightBotPose.timestampSeconds,
                        VecBuilder.fill(30.0, 30.0, 30.0));
            } else {
                double antiTrust = -69.0 * ta + 14.83;
                if (antiTrust <= 10.0) {
                    antiTrust = 10.0;
                }

                m_poseEstimator.addVisionMeasurement(limelightBotPose.pose, limelightBotPose.timestampSeconds,
                        VecBuilder.fill(antiTrust, antiTrust, antiTrust));
            }
        }

    }

    // private void updateWithNote(String limeightName, int noteID) {

    //     double tx = LimelightHelpers.getTX(limeightName);
    //     double ty = LimelightHelpers.getTY(limeightName);
    //     double yAdj = (ty + 0.006461 * tx * tx) / (0.000144 * tx * tx + 1);
    //     double distance = m_noteDistance.get(yAdj) * 0.0254;

    //     if (distance >= 0) {
    //         SmartDashboard.putNumber("DistanceToNote", distance);
    //         Rotation2d robotAngle = m_drivetrain.getGyro();
    //         double visionAngle = (robotAngle.getRadians() - Math.toRadians(tx));
    //         SmartDashboard.putNumber("Vision Angle", visionAngle);
    //         Translation2d noteToRobot = new Translation2d(distance, new Rotation2d(visionAngle));
    //         Pose2d calculatedPose = new Pose2d(VisionConstants.kNoteIDs[noteID].plus(noteToRobot), robotAngle);

    //         SmartDashboard.putNumber("NoteCalcX", calculatedPose.getX());
    //         SmartDashboard.putNumber("NoteCalcY", calculatedPose.getY());
    //         SmartDashboard.putNumber("NoteVecX", noteToRobot.getX() * 39.37);
    //         SmartDashboard.putNumber("NoteVecY", noteToRobot.getY() * 39.37);

    //         double latency = LimelightHelpers.getLatency_Capture(limeightName)
    //                 + LimelightHelpers.getLatency_Pipeline(limeightName);

    //         latency /= 1000.0;

    //         double timestamp = Timer.getFPGATimestamp() - latency;

    //         if(calculatedPose.getTranslation().getDistance(getPose().getTranslation())<= 2.5){
    //             m_noteEstimator.addVisionMeasurement(calculatedPose, timestamp, VecBuilder.fill(2.0, 2.0, 999999));
    //         }

    //     }

    // }

    // public void trackNote(int noteID) {
    //     m_noteID = noteID;
    //     m_noteEstimator.resetPosition(m_drivetrain.getGyro(), m_drivetrain.getModulePositions(), getPose());
    //     m_trackNote = true;
    // }


    // public void trackSneakNote(int noteID) {
    //     var alliance = DriverStation.getAlliance();
    //     boolean redAlliance = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    //     if(redAlliance){noteID+=2;}
    //     m_noteID = noteID;
    //     // m_noteEstimator.resetPosition(m_drivetrain.getGyro(), m_drivetrain.getModulePositions(), getPose());
    //     m_trackNote = true;
    // }

    // public void stopNoteTracking() {
    //     m_trackNote = false;
    // }

    private void updateShuffleboard() {
        Pose2d pose = getPose();
        SmartDashboard.putNumber("PoseEstX", pose.getX());
        SmartDashboard.putNumber("PoseEstY", pose.getY());
        SmartDashboard.putNumber("PoseEstRot", pose.getRotation().getRadians());

        m_field.setRobotPose(pose);
    }

    public Pose2d getPose() {
            return m_poseEstimator.getEstimatedPosition();

    }

    public void setAuto(boolean auto) {
        m_auto = auto;
    }

    public boolean inside(Translation2d[] bounds, boolean onEdge) {
        Pose2d currentPose = getPose();
        double xMin = Math.min(bounds[0].getX(), bounds[1].getX());
        double xMax = Math.max(bounds[0].getX(), bounds[1].getX());
        double yMin = Math.min(bounds[0].getY(), bounds[1].getY());
        double yMax = Math.max(bounds[0].getY(), bounds[1].getY());
        return ((currentPose.getX() > xMin && currentPose.getX() < xMax)
                || (onEdge && (currentPose.getX() >= xMin && currentPose.getX() <= xMax))
                        &&
                        (currentPose.getY() > yMin && currentPose.getY() < yMax)
                || (onEdge && (currentPose.getY() >= yMin && currentPose.getY() <= yMax)));
    }

    public void resetOdometry(Pose2d pose) {
        m_drivetrain.resetOdometry(pose.getRotation().times(-1.0));
        m_poseEstimator.resetPosition(m_drivetrain.getGyro().times(1.0), m_drivetrain.getModulePositions(), pose);
    }

    public void updatePose() {
        m_drivetrain.resetOdometry(getPose().getRotation().times(-1.0));
    }

}
