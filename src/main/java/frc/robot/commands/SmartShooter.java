package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pitcher;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.MathUtils;

public class SmartShooter extends Command {
    private final Shooter m_shooter;
    private final Drivetrain m_robotDrive;
    private final Pitcher m_pitcher;

    private final PIDController  m_pid = new PIDController(0.085,0.02,0.0);
    private InterpolatingDoubleTreeMap m_pitchTable = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap m_velocityTable = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap m_timeTable = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap m_distTable = new InterpolatingDoubleTreeMap();
    private final CommandXboxController m_controller;
    private double manualHoodValue = 5.0;
    private boolean manualHoodOverride = false;
    private double manualVelocityValue = 70.0;
    private boolean manualVelocityOverride = false;
    private boolean m_feedStarted = false;

    private final Timer m_timer = new Timer();

    private final SlewRateLimiter m_rotFilter = new SlewRateLimiter(12.0);
    private final SlewRateLimiter m_pitchFilter = new SlewRateLimiter(20.0);
    private final SlewRateLimiter m_velocityFilter = new SlewRateLimiter(300.0);

    public SmartShooter(Shooter shooter, Drivetrain robotDrive, Pitcher pitcher, CommandXboxController controller){
        m_shooter = shooter;
        m_robotDrive = robotDrive;
        m_pitcher = pitcher;
        m_controller = controller;

        m_pid.setIntegratorRange(-0.15, 0.15);

        m_distTable = MathUtils.pointsToTreeMap(ShooterConstants.kDistTable);

        m_pitchTable = MathUtils.pointsToTreeMap(ShooterConstants.kPitchTable);

        m_velocityTable = MathUtils.pointsToTreeMap(ShooterConstants.kVelocityTable);

        m_timeTable = MathUtils.pointsToTreeMap(ShooterConstants.kTimeTable);
        
        addRequirements(m_robotDrive, m_shooter);

    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        m_pid.reset();
        SmartDashboard.putBoolean("Manual Velocity Override", manualVelocityOverride);
        SmartDashboard.putNumber("Set Velocity Adjust", manualVelocityValue);

        SmartDashboard.putBoolean("Manual Hood Override", manualHoodOverride);
        SmartDashboard.putNumber("Set Hood Adjust", manualHoodValue);
        m_feedStarted = false;
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        double tx = LimelightHelpers.getTX("limelight-april");
        double ty = LimelightHelpers.getTY("limelight-april");
        boolean tv = LimelightHelpers.getTV("limelight-april");
        double desiredRot = -MathUtils.inputTransform(m_controller.getRightX())* DriveConstants.kMaxAngularSpeed;

        SmartDashboard.putNumber("Limelight Ty", ty);

        Translation2d relGoalPose = compRelGoalPose(ty, tx);

        double pidAngle = (relGoalPose.getAngle().getDegrees());
        
        double goalDistance = relGoalPose.getDistance(new Translation2d());

        boolean ready = m_timer.get() >= 0.05;

        if(tv && ready){
            desiredRot = m_pid.calculate(pidAngle);
        }
/*             if(goodToShoot && !m_feedStarted){
                m_indexer.run(0.4);
                m_feeder.run(0.4);
                m_feedStarted = true;
            }
            else if(m_feedStarted){
                m_indexer.run(0.4);
                m_feeder.run(0.4);
            }
            else{
                m_indexer.stop();
                m_feeder.stop();
            }
        }
        else{
            m_indexer.stop();
            m_feeder.stop();
        }
 */
        manualHoodOverride = SmartDashboard.getBoolean("Manual Hood Override", false);
        manualVelocityOverride = SmartDashboard.getBoolean("Manual Velocity Override", false);

        if(manualHoodOverride && manualVelocityOverride){
            manualHoodValue = SmartDashboard.getNumber("Set Hood Adjust", 0);
            manualVelocityValue = SmartDashboard.getNumber("Set Velocity Adjust", 70.0);
            m_pitcher.pitchToAngle(manualHoodValue);
            m_shooter.run(manualVelocityValue);
        }
        else if(manualHoodOverride){
            manualHoodValue = SmartDashboard.getNumber("Set Hood Adjust", 0);
            m_pitcher.pitchToAngle(manualHoodValue);
        }
        else if(manualVelocityOverride){
            manualVelocityValue = SmartDashboard.getNumber("Set Velocity Adjust", 70.0);
            m_shooter.run(manualVelocityValue);
        }
        else if(ready){
            if(tv){
            m_pitcher.pitchToAngle(m_pitchFilter.calculate(m_pitchTable.get(goalDistance*39.37)));
            m_shooter.run(m_velocityFilter.calculate(m_velocityTable.get(goalDistance*39.37))); 
            }
            else{
            m_pitcher.pitchToAngle(2.0);
            m_shooter.run(m_velocityFilter.calculate(50.0));
            }
        }

            var alliance = DriverStation.getAlliance();

    double xInput = -m_controller.getLeftY();
    double yInput =  -m_controller.getLeftX();

    if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red){
      xInput = -xInput;
      yInput = -yInput;
    }

    double desiredTrans[] = MathUtils.inputTransform(xInput, yInput);
        
        double maxLinear = DriveConstants.kMaxSpeedMetersPerSecond;

        desiredTrans[0] *= maxLinear;
        desiredTrans[1] *= maxLinear;

        //double desiredRot = -MathUtils.inputTransform(m_controller.getRightX())* DriveConstants.kMaxAngularSpeed;

        //Translation2d rotAdj= desiredTranslation.rotateBy(new Rotation2d(-Math.PI/2.0)).times(desiredRot*0.05);

        //desiredTranslation = desiredTranslation.plus(rotAdj);

        m_robotDrive.drive(desiredTrans[0], desiredTrans[1],m_rotFilter.calculate(desiredRot),true,true);
        

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
        m_shooter.stop();
/*                         m_indexer.stop();
                m_feeder.stop(); */
        m_pitcher.pitchToAngle(3.0);
    }

    public Translation2d compRelGoalPose(double ty, double tx){
        double distToGoal = m_distTable.get(ty)/39.37;

        Translation2d goalRelPose = new Translation2d(distToGoal*Math.cos(Math.toRadians(tx)),distToGoal*Math.sin(Math.toRadians(tx)));

        SmartDashboard.putNumber("Rel Goal X", goalRelPose.getX());
        SmartDashboard.putNumber("Rel Goal Y", goalRelPose.getY());

        double rx = m_robotDrive.getChassisSpeed().vxMetersPerSecond+m_robotDrive.getChassisAccel().ax*0.025;
        double ry = m_robotDrive.getChassisSpeed().vyMetersPerSecond+m_robotDrive.getChassisAccel().ay*0.025;

        double shotTime = m_timeTable.get(distToGoal);

        return new Translation2d(goalRelPose.getX()-rx*shotTime,goalRelPose.getY()+ry*shotTime);
    }
    


}
