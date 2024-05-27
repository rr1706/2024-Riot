package frc.robot.commands;

import java.util.function.Supplier;

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
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.PitcherConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pitcher;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.MathUtils;

public class SmartShootByPose extends Command {
    private final Shooter m_shooter;
    private final Drivetrain m_robotDrive;
    private final Pitcher m_pitcher;
    private Supplier<Pose2d> getPose;

    private final PIDController m_pid = new PIDController(0.115, 0.010, 0.0);
    private InterpolatingDoubleTreeMap m_pitchTable = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap m_velocityTable = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap m_timeTable = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap m_feedPitch = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap m_feedVelocity = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap m_feedTime = new InterpolatingDoubleTreeMap();
    private final CommandXboxController m_controller;
    private double manualHoodValue = 5.0;
    private boolean manualOverride = false;
    private double manualVelocityValue = 70.0;
    private double manualSpinDiff = 0.0;

    private double pitchOffset = 0.0;

    private final Timer m_timer = new Timer();

    private final SlewRateLimiter m_pitchFilter = new SlewRateLimiter(100.0);
    private final SlewRateLimiter m_velocityFilter = new SlewRateLimiter(500.0);

    public SmartShootByPose(Shooter shooter, Drivetrain robotDrive, Pitcher pitcher, CommandXboxController controller,
            Supplier<Pose2d> getPose) {
        m_shooter = shooter;
        m_robotDrive = robotDrive;
        m_pitcher = pitcher;
        m_controller = controller;

        this.getPose = getPose;

        m_pid.setIntegratorRange(-0.1, 0.1);

        m_pitchTable = MathUtils.pointsToTreeMap(ShooterConstants.kPitchTable);
        m_feedPitch = MathUtils.pointsToTreeMap(ShooterConstants.kFeedPitch);
        m_velocityTable = MathUtils.pointsToTreeMap(ShooterConstants.kVelocityTable);
        m_feedVelocity = MathUtils.pointsToTreeMap(ShooterConstants.kFeedVelocity);
        m_timeTable = MathUtils.pointsToTreeMap(ShooterConstants.kTimeTable);
        m_feedTime = MathUtils.pointsToTreeMap(ShooterConstants.kFeedTime);
        addRequirements(m_robotDrive, m_shooter);

    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        m_pid.reset();
        SmartDashboard.putNumber("Set Hood Adjust", manualHoodValue);
        SmartDashboard.putNumber("Set Velocity Adjust", manualVelocityValue);
        SmartDashboard.putNumber("Set Spin Diff", manualSpinDiff);

        SmartDashboard.putBoolean("Manual Override", manualOverride);
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        var alliance = DriverStation.getAlliance();

        boolean redAlliance = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;

        if (redAlliance) {
            pitchOffset = 0.10;
        } else {
            pitchOffset = 0.0;
        }

        Translation2d goalLocation;
        boolean feedShot = false;

        if (redAlliance) {
            goalLocation = GoalConstants.kRedGoal;
            if (getPose.get().getTranslation().getX() <= 4.0) {
                goalLocation = GoalConstants.kMidFeed;
                feedShot = true;
            } else if (getPose.get().getTranslation().getX() <= 9.0) {

                goalLocation = GoalConstants.kRedFeed;
                feedShot = true;
            }
        } else {
            goalLocation = GoalConstants.kBlueGoal;
            if (getPose.get().getTranslation().getX() >= 12.5) {
                goalLocation = GoalConstants.kMidFeed;
                feedShot = true;
            } else if (getPose.get().getTranslation().getX() >= 7.5) {

                goalLocation = GoalConstants.kBlueFeed;
                feedShot = true;
            }
        }

        goalLocation = compForMovement(goalLocation, feedShot);

        double desiredRot = -MathUtils.inputTransform(m_controller.getRightX()) * DriveConstants.kMaxAngularSpeed;

        Translation2d toGoal = goalLocation.minus(getPose.get().getTranslation());

        SmartDashboard.putNumber("Goal Angle", toGoal.getAngle().getDegrees());

        double angle = toGoal.getAngle().getRadians();

        double offset = (0.2 / 0.7854) * Math.abs(Math.asin(Math.sin(angle)));

        double pidAngle = -1.0 * toGoal.getAngle().minus(getPose.get().getRotation()).getDegrees();

        double goalDistance = toGoal.getDistance(new Translation2d()) * 39.37;

        offset *= -0.00385 * goalDistance + 1.69;

        SmartDashboard.putNumber("Pitch offset", offset);

        SmartDashboard.putNumber("Pose Distance", goalDistance);
        SmartDashboard.putNumber("Shooter Angle Error", pidAngle);

        desiredRot = m_pid.calculate(pidAngle);

        manualOverride = SmartDashboard.getBoolean("Manual Override", false);

        if (manualOverride) {
            manualHoodValue = SmartDashboard.getNumber("Set Hood Adjust", 0);
            manualVelocityValue = SmartDashboard.getNumber("Set Velocity Adjust", 70.0);
            manualSpinDiff = SmartDashboard.getNumber("Set Spin Diff", 0.0);
            m_pitcher.pitchToAngle(manualHoodValue);
            m_shooter.run(manualVelocityValue, manualSpinDiff);
        } else {
            if (feedShot) {
                m_pitcher.pitchToAngle(m_pitchFilter.calculate(m_feedPitch.get(goalDistance)));
                m_shooter.run(m_velocityFilter.calculate(m_feedVelocity.get(goalDistance)), 0.0);
            } else {
                m_pitcher.pitchToAngle(m_pitchFilter.calculate(m_pitchTable.get(goalDistance)) + pitchOffset);
                m_shooter.run(m_velocityFilter.calculate(m_velocityTable.get(goalDistance)), 30.0);
            }
        }

        double xInput = -m_controller.getLeftY();
        double yInput = -m_controller.getLeftX();

        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            xInput = -xInput;
            yInput = -yInput;
        }

        double desiredTrans[] = MathUtils.inputTransform(xInput, yInput);

        double maxLinear = DriveConstants.kMaxSpeedMetersPerSecond * 0.4;

        desiredTrans[0] *= maxLinear;
        desiredTrans[1] *= maxLinear;

        // double desiredRot = -MathUtils.inputTransform(m_controller.getRightX())*
        // DriveConstants.kMaxAngularSpeed;

        // Translation2d rotAdj= desiredTranslation.rotateBy(new
        // Rotation2d(-Math.PI/2.0)).times(desiredRot*0.05);

        // desiredTranslation = desiredTranslation.plus(rotAdj);

        m_robotDrive.drive(desiredTrans[0], desiredTrans[1], (desiredRot), true, true);

    }

    Translation2d compForMovement(Translation2d goalLocation, boolean feedShot) {

        Translation2d toGoal = goalLocation.minus(getPose.get().getTranslation());

        double rx = m_robotDrive.getFieldRelativeSpeed().vx + m_robotDrive.getFieldRelativeAccel().ax * 0.015;
        double ry = m_robotDrive.getFieldRelativeSpeed().vy + m_robotDrive.getFieldRelativeAccel().ay * 0.015;

        double shotTime;
        if (feedShot) {
            shotTime = m_feedTime.get(toGoal.getDistance(new Translation2d()));
        } else {

            shotTime = m_timeTable.get(toGoal.getDistance(new Translation2d()));
        }
        return new Translation2d(goalLocation.getX() - rx * shotTime, goalLocation.getY() - ry * shotTime);
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        m_shooter.stop();
        /*
         * m_indexer.stop();
         * m_feeder.stop();
         */
        m_pitcher.pitchToAngle(PitcherConstants.kHome);
    }

}
