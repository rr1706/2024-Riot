package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pitcher;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.MathUtils;

public class AutoShooter extends Command {
    private final Shooter m_shooter;
    private final Pitcher m_pitcher;
    private final Drivetrain m_robotDrive;
    private final PIDController  m_pid = new PIDController(0.1,0.0,0.0);
    private final InterpolatingDoubleTreeMap m_pitchTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_velocityTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_timeTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_distTable = new InterpolatingDoubleTreeMap();
    private double manualHoodValue = 5.0;
    private boolean manualHoodOverride = false;
    private double manualVelocityValue = 70.0;
    private boolean manualVelocityOverride = false;
    private boolean m_feedStarted = false;

    private final SlewRateLimiter m_rotFilter = new SlewRateLimiter(12.0);
    private final SlewRateLimiter m_pitchFilter = new SlewRateLimiter(20.0);
    private final SlewRateLimiter m_velocityFilter = new SlewRateLimiter(300.0);

    public AutoShooter(Shooter shooter, Pitcher pitcher, Drivetrain drive){
        m_shooter = shooter;
        m_pitcher = pitcher;
        m_robotDrive = drive;

        m_distTable.put(16.75,38.0);
        m_distTable.put(12.15, 52.25);
        m_distTable.put(8.95, 56.0);
        m_distTable.put(6.11, 44.625+38.0);
        m_distTable.put(3.76, 66.0+38.0);
        m_distTable.put(2.17, 82.0+38.0);
        m_distTable.put(0.23, 107.0+38.0);
        m_distTable.put(-1.33, 128.5+38.0);
        m_distTable.put(-2.49, 150.5+38.0);
        m_distTable.put(-3.57, 174.0+38.0);
        m_distTable.put(-3.99, 196.0+38.0);
        m_distTable.put(-4.7, 221.5+38.0);

        m_pitchTable.put(41.0,20.0);
        m_pitchTable.put(53.68,18.50);
        m_pitchTable.put(63.77,16.67);
        m_pitchTable.put(85.77,14.97);
        m_pitchTable.put(100.2,13.41);
        m_pitchTable.put(120.0,12.40);
        m_pitchTable.put(140.0,10.54);
        m_pitchTable.put(160.5,9.05);
        m_pitchTable.put(171.0,7.86);
        m_pitchTable.put(179.0,7.27);
        m_pitchTable.put(200.0,6.51);

        m_velocityTable.put(41.0,47.0);
        m_velocityTable.put(53.68,47.0);
        m_velocityTable.put(63.77,47.0);
        m_velocityTable.put(85.77,47.0);
        m_velocityTable.put(100.2,51.33);
        m_velocityTable.put(120.0,53.25);
        m_velocityTable.put(140.0,56.03);
        m_velocityTable.put(160.5,60.09);
        m_velocityTable.put(171.0,70.13);
        m_velocityTable.put(179.0,80.54);
        m_velocityTable.put(200.0,85.6);

        m_timeTable.put(1.0, 0.3);
        m_timeTable.put(3.0, 0.35);
        m_timeTable.put(5.0, 0.4);
        
        addRequirements(m_shooter);

    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        SmartDashboard.putBoolean("Manual Velocity Override", manualVelocityOverride);
        SmartDashboard.putNumber("Set Velocity Adjust", manualVelocityValue);

        SmartDashboard.putBoolean("Manual Hood Override", manualHoodOverride);
        SmartDashboard.putNumber("Set Hood Adjust", manualHoodValue);
        m_feedStarted = false;
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        double tx = LimelightHelpers.getTX("limelight-april");
        double ty = LimelightHelpers.getTY("limelight-april");

        Translation2d relGoalPose = compRelGoalPose(ty, tx);

        
        double goalDistance = relGoalPose.getDistance(new Translation2d());




            m_pitcher.pitchToAngle(m_pitchFilter.calculate(m_pitchTable.get(goalDistance*39.37)));
            m_shooter.run(m_velocityFilter.calculate(m_velocityTable.get(goalDistance*39.37)));
        

/*     m_robotDrive.drive(m_slewX.calculate(
        -inputTransform(m_controller.getLeftY()))
        * DriveConstants.kMaxSpeedMetersPerSecond,
        m_slewY.calculate(
            -inputTransform(m_controller.getLeftX()))
            * DriveConstants.kMaxSpeedMetersPerSecond,
        m_slewRot.calculate(-inputTransform(m_controller.getRightX()))
            * DriveConstants.kMaxAngularSpeed,
        fieldOrient); */

    }
    
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        m_shooter.stop();
/*                         m_indexer.stop();
                m_feeder.stop(); */
        m_pitcher.pitchToAngle(2.0);
    }

    public Translation2d compRelGoalPose(double ty, double tx){
        double distToGoal = m_distTable.get(ty)/39.37;

        Translation2d goalRelPose = new Translation2d(distToGoal*Math.cos(Math.toRadians(tx)),distToGoal*Math.sin(Math.toRadians(tx)));

        SmartDashboard.putNumber("Rel Goal X", goalRelPose.getX());
        SmartDashboard.putNumber("Rel Goal Y", goalRelPose.getY());

        double rx = m_robotDrive.getChassisSpeed().vxMetersPerSecond;
        double ry = m_robotDrive.getChassisSpeed().vyMetersPerSecond;

        double shotTime = m_timeTable.get(distToGoal);

        return new Translation2d(goalRelPose.getX()-rx*shotTime,goalRelPose.getY()+ry*shotTime);
    }
    


}
