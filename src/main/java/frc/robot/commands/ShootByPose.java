package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Pitcher;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.MathUtils;

public class ShootByPose extends Command {
    private final Shooter m_shooter;
    private final Pitcher m_pitcher;
    private final Translation2d m_location;
    //private final PIDController  m_pid = new PIDController(0.1,0.0,0.0);
    private final Timer m_timer = new Timer();
    private final InterpolatingDoubleTreeMap m_pitchTable = MathUtils.pointsToTreeMap(ShooterConstants.kPitchTable);
    private final InterpolatingDoubleTreeMap m_velocityTable = MathUtils.pointsToTreeMap(ShooterConstants.kVelocityTable);

    public ShootByPose(Shooter shooter, Pitcher pitcher, Translation2d location){
        m_shooter = shooter;
        m_pitcher = pitcher;
        m_location = location;

        addRequirements(shooter, pitcher);

    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        Translation2d goal = new Translation2d(-1.50/39.37, 218.42/39.37);

        double distance = (m_location.getDistance(goal))*39.37-14.5;

        SmartDashboard.putNumber("Pose Distance", distance);

        m_pitcher.pitchToAngle(m_pitchTable.get(distance));
        m_shooter.run(m_velocityTable.get(distance));

        SmartDashboard.putBoolean("DrivingByController", true);
    }
    
    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_pitcher.pitchToAngle(2.0);
        m_timer.stop();
    }

    


}
