package frc.robot.subsystems;

import java.util.Optional;

import javax.naming.LimitExceededException;
import javax.swing.text.StyledEditorKit.BoldAction;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

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
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.utilities.MathUtils;

public class PoseEstimator extends SubsystemBase {
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final Drivetrain m_drivetrain;
    private final Field2d m_field = new Field2d();
    private boolean m_auto = true;
    private boolean m_autoRotate = false;
    private InterpolatingDoubleTreeMap m_timeTable = new InterpolatingDoubleTreeMap();


    public PoseEstimator(Drivetrain drivetrain) {
        m_timeTable = MathUtils.pointsToTreeMap(ShooterConstants.kTimeTable);
        m_drivetrain = drivetrain;
        m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kSwerveKinematics,
                drivetrain.getGyro(),
                drivetrain.getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.229, 0.229, 0.229),
                VecBuilder.fill(10, 10, 10));
        SmartDashboard.putData("Field", m_field);
        SmartDashboard.putBoolean("Reset Pose", false);
    }

    @Override
    public void periodic() {
        updatePoseEstimator(false);
        updateShuffleboard();
    }

    private void updatePoseEstimator(boolean force) {

        //double velocity = MathUtils.pythagorean(m_drivetrain.getChassisSpeed().vxMetersPerSecond,
                //m_drivetrain.getChassisSpeed().vyMetersPerSecond;
        //double angularVelocity = m_drivetrain.getChassisSpeed().omegaRadiansPerSecond;

        // double yAdj = 0.945*(limelightBotPose.getY())+.305;

        // limelightBotPose = new Pose2d(limelightBotPose.getX(), yAdj,
        // limelightBotPose.getRotation());

        //Pose2d currentPose = getPose();

        SmartDashboard.putBoolean("Auto Pose", m_auto);

        // boolean m_overide = SmartDashboard.getBoolean("Set Pose Est", false);

        m_poseEstimator.updateWithTime(Timer.getFPGATimestamp(), m_drivetrain.getGyro(),
                m_drivetrain.getModulePositions());
        updateWithVision("limelight-april");
        
        if(!m_auto){
            updateWithVision("limelight-back");
        }

    }

    private void updateWithVision(String limelightName) {
        double ta = LimelightHelpers.getTA(limelightName);
        PoseEstimate limelightBotPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        int validTagCount = limelightBotPose.tagCount;
        boolean slowRotate = m_drivetrain.getChassisSpeed().omegaRadiansPerSecond <= 4*Math.PI;

        if ((validTagCount >= 2.0 && ta >= 0.030) && slowRotate) {
            if (m_auto) {
                double antiTrust = 10.0;
                m_poseEstimator.addVisionMeasurement(limelightBotPose.pose, limelightBotPose.timestampSeconds, VecBuilder.fill(antiTrust, antiTrust, antiTrust));
            } else {
                double antiTrust = -150.0*ta+10.0;
                if(antiTrust <= 2.0){antiTrust = 2.0;}
                m_poseEstimator.addVisionMeasurement(limelightBotPose.pose, limelightBotPose.timestampSeconds, VecBuilder.fill(antiTrust, antiTrust, antiTrust));

            }
        } else if ((validTagCount == 1 && ta >= 0.070) && !m_auto && slowRotate) {
                double antiTrust = -69.0*ta+14.83;
            if(antiTrust <= 5.0){antiTrust = 5.0;}
            m_poseEstimator.addVisionMeasurement(limelightBotPose.pose, limelightBotPose.timestampSeconds, VecBuilder.fill(antiTrust, antiTrust, antiTrust));
        }
    }

        private void updateWithVision2(String limelightName) {
        LimelightHelpers.SetRobotOrientation(limelightName, m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        double limelightLatency = LimelightHelpers.getLatency_Pipeline(limelightName)
                + LimelightHelpers.getLatency_Capture(limelightName);
        limelightLatency = limelightLatency / 1000.0;
        double ta = LimelightHelpers.getTA(limelightName);
        PoseEstimate limelightBotPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        int validTagCount = limelightBotPose.tagCount;
        if ((validTagCount >= 2.0)) {
            if (m_auto) {
                double antiTrust = -300.0*ta+22.0;
                if(antiTrust <= 5.0){antiTrust = 5.0;}
                m_poseEstimator.addVisionMeasurement(limelightBotPose.pose, limelightBotPose.timestampSeconds, VecBuilder.fill(antiTrust, antiTrust, 999999));
            } else {
                double antiTrust = -150.0*ta+10.0;
                if(antiTrust <= 0.5){antiTrust = 0.5;}
                m_poseEstimator.addVisionMeasurement(limelightBotPose.pose, limelightBotPose.timestampSeconds, VecBuilder.fill(antiTrust, antiTrust, 999999));

            }
        } else if ((validTagCount == 1) && !m_auto) {
                double antiTrust = -69.0*ta+14.83;
            m_poseEstimator.addVisionMeasurement(limelightBotPose.pose, limelightBotPose.timestampSeconds, VecBuilder.fill(antiTrust, antiTrust, 999999));
        }
    }

    private void updateShuffleboard() {
        Pose2d pose = getPose();
        SmartDashboard.putNumber("PoseEstX", pose.getX());
        SmartDashboard.putNumber("PoseEstY", pose.getY());
        SmartDashboard.putNumber("PoseEstRot", pose.getRotation().getRadians());

        if (SmartDashboard.getBoolean("Reset Pose", false)) {
            updatePoseEstimator(true);
        }

        m_field.setRobotPose(pose);
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

Translation2d compForMovement(Translation2d goalLocation) {

    Translation2d toGoal = goalLocation.minus(getPose().getTranslation());

    double rx = m_drivetrain.getFieldRelativeSpeed().vx + m_drivetrain.getFieldRelativeAccel().ax * 0.030;
    double ry = m_drivetrain.getFieldRelativeSpeed().vy + m_drivetrain.getFieldRelativeAccel().ay * 0.030;
        
    double shotTime = m_timeTable.get(toGoal.getDistance(new Translation2d()));

    return new Translation2d(goalLocation.getX() - rx * shotTime, goalLocation.getY() - ry * shotTime);
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

}
