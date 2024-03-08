package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.utilities.MathUtils;

public class PoseEstimator extends SubsystemBase {
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final Drivetrain m_drivetrain;
    private final Field2d m_field = new Field2d();
    public PoseEstimator(Drivetrain drivetrain){
        m_drivetrain = drivetrain;
        m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kSwerveKinematics,
             drivetrain.getGyro(), 
             drivetrain.getModulePositions(), 
             new Pose2d(),
             VecBuilder.fill(0.229, 0.229, 0.0), 
             VecBuilder.fill(10, 10, 10));
             SmartDashboard.putData("Field", m_field);
             SmartDashboard.putBoolean("Reset Pose",false);
    }
    @Override
    public void periodic() {
        updatePoseEstimator(false);
        updateShuffleboard();
    }

    private void updatePoseEstimator(boolean force) {
        double limelightLatency = LimelightHelpers.getLatency_Pipeline("limelight-april") + LimelightHelpers.getLatency_Capture("limelight-april");
        limelightLatency = limelightLatency/1000.0;
        double timestamp =  Timer.getFPGATimestamp() - limelightLatency;
        double velocity = MathUtils.pythagorean(m_drivetrain.getChassisSpeed().vxMetersPerSecond, m_drivetrain.getChassisSpeed().vyMetersPerSecond);
        double angularVelocity = m_drivetrain.getChassisSpeed().omegaRadiansPerSecond;
        Pose2d limelightBotPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-april");

        double yAdj = 0.945*(limelightBotPose.getY())+.305;

        limelightBotPose = new Pose2d(limelightBotPose.getX(), yAdj, limelightBotPose.getRotation());

        Pose2d currentPose = getPose();
        boolean validSolution = LimelightHelpers.getTV("limelight-april");

        LimelightResults results = LimelightHelpers.getLatestResults("limelight-april");
        int validTagCount = results.targetingResults.targets_Fiducials.length;

        SmartDashboard.putNumber("Tags Visible", validTagCount);

        SmartDashboard.putNumber("VisionX", limelightBotPose.getX());
        SmartDashboard.putNumber("VisionY", limelightBotPose.getY());

        boolean updatePose = ((currentPose.getTranslation().getDistance(limelightBotPose.getTranslation()) <= VisionConstants.kPoseErrorAcceptance) && validSolution  && limelightBotPose.getTranslation().getDistance(currentPose.getTranslation()) >= 0.05 && velocity <= 4.0 && angularVelocity <= 0.5 * Math.PI);

        boolean withinZOne = true;


        //boolean m_overide = SmartDashboard.getBoolean("Set Pose Est", false);

        m_poseEstimator.updateWithTime(Timer.getFPGATimestamp(), m_drivetrain.getGyro(), m_drivetrain.getModulePositions());
        if ((updatePose && validTagCount == 3) || (updatePose && withinZOne && validTagCount == 2.0)||force) {
                    m_poseEstimator.addVisionMeasurement(limelightBotPose, timestamp, VecBuilder.fill(10.0, 10.0, 10.0));
        } 
    }
        

    private void updateShuffleboard() {
        Pose2d pose = getPose();
        SmartDashboard.putNumber("PoseEstX", pose.getX());
        SmartDashboard.putNumber("PoseEstY", pose.getY());
        SmartDashboard.putNumber("PoseEstRot", pose.getRotation().getRadians());


        if(SmartDashboard.getBoolean("Reset Pose", false)){
            updatePoseEstimator(true);
        }

        m_field.setRobotPose(pose);
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public boolean inside(Translation2d[] bounds, boolean onEdge) {
        Pose2d currentPose = getPose();
        double xMin = Math.min(bounds[0].getX(), bounds[1].getX());
        double xMax = Math.max(bounds[0].getX(), bounds[1].getX());
        double yMin = Math.min(bounds[0].getY(), bounds[1].getY());
        double yMax = Math.max(bounds[0].getY(), bounds[1].getY());
        return (
            (currentPose.getX() > xMin && currentPose.getX() < xMax) || (onEdge && (currentPose.getX() >= xMin && currentPose.getX() <= xMax)) 
            && 
            (currentPose.getY() > yMin && currentPose.getY() < yMax) || (onEdge && (currentPose.getY() >= yMin && currentPose.getY() <= yMax))
        );
    }

    public void resetOdometry(Pose2d pose) {
        m_drivetrain.resetOdometry(pose.getRotation().times(-1.0));
        m_poseEstimator.resetPosition(m_drivetrain.getGyro().times(1.0), m_drivetrain.getModulePositions(), pose);
    }

}
