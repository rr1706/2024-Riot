package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pitcher;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.MathUtils;

public class AutoSmartShooter extends Command {
    private final Shooter m_shooter;
    private final Drivetrain m_robotDrive;
    private final Pitcher m_pitcher;
    private Supplier<Pose2d> getPose;
    private boolean m_autoRotate = false;
    private Rotation2d m_desiredRot = new Rotation2d();
    private SlewRateLimiter m_rotSmooth = new SlewRateLimiter(0.25);

    private InterpolatingDoubleTreeMap m_pitchTable = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap m_velocityTable = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap m_timeTable = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap m_feedPitch = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap m_feedVelocity = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap m_feedTime = new InterpolatingDoubleTreeMap();

    private final Timer m_timer = new Timer();

    private final SlewRateLimiter m_pitchFilter = new SlewRateLimiter(60.0);
    private final SlewRateLimiter m_velocityFilter = new SlewRateLimiter(400.0);

    public AutoSmartShooter(Shooter shooter, Drivetrain robotDrive, Pitcher pitcher,
            Supplier<Pose2d> getPose) {
        m_shooter = shooter;
        m_robotDrive = robotDrive;
        m_pitcher = pitcher;

        this.getPose = getPose;

        m_pitchTable = MathUtils.pointsToTreeMap(ShooterConstants.kPitchTable);
        m_feedPitch = MathUtils.pointsToTreeMap(ShooterConstants.kFeedPitch);
        m_velocityTable = MathUtils.pointsToTreeMap(ShooterConstants.kVelocityTable);
        m_feedVelocity = MathUtils.pointsToTreeMap(ShooterConstants.kFeedVelocity);
        m_timeTable = MathUtils.pointsToTreeMap(ShooterConstants.kTimeTable);
        m_feedTime = MathUtils.pointsToTreeMap(ShooterConstants.kFeedTime);
        
        addRequirements(m_shooter);

    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();

        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

        m_autoRotate = true;

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

        m_desiredRot = new Rotation2d(m_rotSmooth.calculate(toGoal.getAngle().getRadians()));

        SmartDashboard.putNumber("Goal Angle", toGoal.getAngle().getDegrees());

        double angle = toGoal.getAngle().getRadians();

        double offset = (0.2 / 0.7854) * Math.abs(Math.asin(Math.sin(angle)));

        double goalDistance = toGoal.getDistance(new Translation2d()) * 39.37;

        offset *= -0.00385 * goalDistance + 1.69;

        SmartDashboard.putNumber("Pitch offset", offset);

        SmartDashboard.putNumber("Pose Distance", goalDistance);

        m_pitcher.pitchToAngle(m_pitchFilter.calculate(m_pitchTable.get(goalDistance)) + offset);
        m_shooter.run(m_velocityFilter.calculate(m_velocityTable.get(goalDistance)),30.0);

        

    }

    public Optional<Rotation2d> getRotationTargetOverride(){
    // Some condition that should decide if we want to override rotation
    if(m_autoRotate) {
        return Optional.of(m_desiredRot);
    } else {
        // return an empty optional when we don't want to override the path's rotation
        return Optional.empty();
    }
    }

    Translation2d compForMovement(Translation2d goalLocation) {

        Translation2d toGoal = goalLocation.minus(getPose.get().getTranslation());

        double rx = m_robotDrive.getFieldRelativeSpeed().vx;
        double ry = m_robotDrive.getFieldRelativeSpeed().vy;
        double shotTime = m_feedTime.get(toGoal.getDistance(new Translation2d()));
        return new Translation2d(goalLocation.getX() - rx * shotTime, goalLocation.getY() - ry * shotTime);
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        m_shooter.stop();
        m_autoRotate = false;
        /*
         * m_indexer.stop();
         * m_feeder.stop();
         */
        m_pitcher.pitchToAngle(2.0);
    }

}
