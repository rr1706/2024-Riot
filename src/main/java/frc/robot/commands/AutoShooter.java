package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pitcher;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.MathUtils;

public class AutoShooter extends Command {
    private final Shooter m_shooter;
    private final Drivetrain m_robotDrive;
    private final Pitcher m_pitcher;
    private final PIDController  m_pid = new PIDController(0.1,0.0,0.0);
    private final Timer m_timer = new Timer();
    private final InterpolatingDoubleTreeMap m_pitchTable = MathUtils.pointsToTreeMap(ShooterConstants.kPitchTable);
    private final InterpolatingDoubleTreeMap m_velocityTable = MathUtils.pointsToTreeMap(ShooterConstants.kVelocityTable);
    private final InterpolatingDoubleTreeMap m_distTable = MathUtils.pointsToTreeMap(ShooterConstants.kDistTable);
    private double manualHoodValue = 5.0;
    private boolean manualHoodOverride = false;
    private double manualVelocityValue = 70.0;
    private boolean manualVelocityOverride = false;
    public AutoShooter(Shooter shooter, Pitcher pitcher, Drivetrain robotDrive){
        m_shooter = shooter;
        m_robotDrive = robotDrive;
        m_pitcher = pitcher;
        addRequirements(m_robotDrive, shooter, pitcher);

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
        // TODO Auto-generated method stub
        double tx = LimelightHelpers.getTX("limelight-april");

        double distance = m_distTable.get(LimelightHelpers.getTY("limelight-april"));

        SmartDashboard.putNumber("Distance", distance);

        boolean tv = LimelightHelpers.getTV("limelight-april");

        double desiredRot = 0.0;

        boolean ready = m_timer.get() >=0.05;

        if(tv && ready){
            desiredRot = m_pid.calculate(tx);
        }

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
            m_pitcher.pitchToAngle(m_pitchTable.get(distance));
            m_shooter.run(m_velocityTable.get(distance));
        }


        //double desiredRot = -MathUtils.inputTransform(m_controller.getRightX())* DriveConstants.kMaxAngularSpeed;

        //Translation2d rotAdj= desiredTranslation.rotateBy(new Rotation2d(-Math.PI/2.0)).times(desiredRot*0.05);

        //desiredTranslation = desiredTranslation.plus(rotAdj);

        m_robotDrive.drive(0,0,desiredRot,true,true);
        

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
