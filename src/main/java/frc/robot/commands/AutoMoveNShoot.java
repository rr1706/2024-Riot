package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pitcher;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.MathUtils;

public class AutoMoveNShoot extends Command {
    private final Shooter m_shooter;
    private final Drivetrain m_robotDrive;
    private final Pitcher m_pitcher;
    private Supplier<Pose2d> getPose;

    private final PIDController m_pid = new PIDController(0.115, 0.010, 0.0);
    private InterpolatingDoubleTreeMap m_pitchTable = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap m_velocityTable = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap m_timeTable = new InterpolatingDoubleTreeMap();
    private double m_xInput = 0.0;
    private double m_yInput = 0.0;
    private SlewRateLimiter m_slew = new SlewRateLimiter(4.0);

    private final Timer m_timer = new Timer();

    private final SlewRateLimiter m_pitchFilter = new SlewRateLimiter(60.0);
    private final SlewRateLimiter m_velocityFilter = new SlewRateLimiter(400.0);

    public AutoMoveNShoot(Shooter shooter, Drivetrain robotDrive, Pitcher pitcher,
            Supplier<Pose2d> getPose, double x, double y) {
        m_shooter = shooter;
        m_robotDrive = robotDrive;
        m_pitcher = pitcher;
        m_xInput = x;
        m_yInput = y;

        this.getPose = getPose;

        m_pid.setIntegratorRange(-0.05, 0.05);

        m_pitchTable = MathUtils.pointsToTreeMap(ShooterConstants.kPitchTable);
        m_velocityTable = MathUtils.pointsToTreeMap(ShooterConstants.kVelocityTable);
        m_timeTable = MathUtils.pointsToTreeMap(ShooterConstants.kTimeTable);
        addRequirements(m_robotDrive, m_shooter);

    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        m_pid.reset();
        m_slew.reset(0.0);
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        var alliance = DriverStation.getAlliance();

        Translation2d goalLocation;

        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            goalLocation = GoalConstants.kRedGoal;

        } else {
            goalLocation = GoalConstants.kBlueGoal;

        }

        goalLocation = compForMovement(goalLocation);

        Translation2d toGoal = goalLocation.minus(getPose.get().getTranslation());

        double pidAngle = -1.0 * toGoal.getAngle().minus(getPose.get().getRotation()).getDegrees();

        double goalDistance = toGoal.getDistance(new Translation2d()) * 39.37;

        double desiredRot = m_pid.calculate(pidAngle);

        m_pitcher.pitchToAngle(m_pitchFilter.calculate(m_pitchTable.get(goalDistance)) + 0.0);
        m_shooter.run(m_velocityFilter.calculate(m_velocityTable.get(goalDistance)), 30.0);

        double xVal;

        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            xVal = -m_xInput;
            m_yInput = -m_yInput;
        } else {
            xVal = m_xInput;
        }

        xVal = m_slew.calculate(xVal);

        double desiredTrans[] = { xVal, m_yInput };

        m_robotDrive.drive(desiredTrans[0], desiredTrans[1], (desiredRot), true, true);

    }

    Translation2d compForMovement(Translation2d goalLocation) {

        Translation2d toGoal = goalLocation.minus(getPose.get().getTranslation());

        double rx = m_robotDrive.getFieldRelativeSpeed().vx;// + m_robotDrive.getFieldRelativeAccel().ax * 0.010;
        double ry = m_robotDrive.getFieldRelativeSpeed().vy;// + m_robotDrive.getFieldRelativeAccel().ay * 0.010;

        double shotTime = m_timeTable.get(toGoal.getDistance(new Translation2d()));
        return new Translation2d(goalLocation.getX() - rx * shotTime, goalLocation.getY() - ry * shotTime);
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        m_shooter.stop();
        m_pitcher.pitchToAngle(2.0);
    }

}
