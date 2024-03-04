package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pitcher;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.MathUtils;

public class AutoShooterByPose extends Command {
    private final Shooter m_shooter;
    private final Pitcher m_pitcher;
    private final Supplier<Pose2d> getPose;
    private final PIDController  m_pid = new PIDController(0.1,0.0,0.0);
    private final Timer m_timer = new Timer();
    private final InterpolatingDoubleTreeMap m_pitchTable = MathUtils.pointsToTreeMap(ShooterConstants.kPitchTable);
    private final InterpolatingDoubleTreeMap m_velocityTable = MathUtils.pointsToTreeMap(ShooterConstants.kVelocityTable);
    private final InterpolatingDoubleTreeMap m_distTable = MathUtils.pointsToTreeMap(ShooterConstants.kDistTable);
    private double manualHoodValue = 5.0;
    private boolean manualHoodOverride = false;
    private double manualVelocityValue = 70.0;
    private boolean manualVelocityOverride = false;
    public AutoShooterByPose(Shooter shooter, Pitcher pitcher, Supplier<Pose2d> getPose){
        m_shooter = shooter;
        m_pitcher = pitcher;
        this.getPose = getPose;

        addRequirements(shooter, pitcher);

    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        SmartDashboard.putBoolean("Manual Velocity Override", manualVelocityOverride);
        SmartDashboard.putNumber("Set Velocity Adjust", manualVelocityValue);

        SmartDashboard.putBoolean("Manual Hood Override", manualHoodOverride);
        SmartDashboard.putNumber("Set Hood Adjust", manualHoodValue);
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stud
        var alliance = DriverStation.getAlliance();

        Translation2d goal = new Translation2d();

        if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red){
            goal = new Translation2d(652.73/39.37, 218.42/39.37);
        }
        else{
            goal = new Translation2d(-1.50/39.37, 218.42/39.37);
        }

        boolean tv = LimelightHelpers.getTV("limelight-april");

        double tx = LimelightHelpers.getTX("limelight-april");

        double distance = (getPose.get().getTranslation().getDistance(goal))*39.37;

        if(tv){
            distance = m_distTable.get(LimelightHelpers.getTY("limelight-april"));
        }

        SmartDashboard.putNumber("Pose Distance", distance);

            m_pitcher.pitchToAngle(m_pitchTable.get(distance));
            m_shooter.run(m_velocityTable.get(distance));


        //double desiredRot = -MathUtils.inputTransform(m_controller.getRightX())* DriveConstants.kMaxAngularSpeed;

        //Translation2d rotAdj= desiredTranslation.rotateBy(new Rotation2d(-Math.PI/2.0)).times(desiredRot*0.05);

        //desiredTranslation = desiredTranslation.plus(rotAdj);


        

/*     m_robotDrive.drive(m_slewX.calculate(
        -inputTransform(m_controller.getLeftY()))
        * DriveConstants.kMaxSpeedMetersPerSecond,
        m_slewY.calculate(
            -inputTransform(m_controller.getLeftX()))
            * DriveConstants.kMaxSpeedMetersPerSecond,
        m_slewRot.calculate(-inputTransform(m_controller.getRightX()))
            * DriveConstants.kMaxAngularSpeed,
        fieldOrient); */

        SmartDashboard.putBoolean("DrivingByController", true);
    }
    
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        m_shooter.run(50.0);
        m_pitcher.pitchToAngle(8.0);
        m_timer.stop();
    }

    


}
