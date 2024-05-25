package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class RotateToAngle extends Command {

    private final Drivetrain m_drive;
    private final double m_angle;
    private final ProfiledPIDController m_pid = new ProfiledPIDController(1.0, 0.00, 0.0,
            new Constraints(2 * Math.PI, 4 * Math.PI));
    private boolean m_finished = false;

    public RotateToAngle(Rotation2d angle, Drivetrain drive) {
        m_drive = drive;
        m_angle = angle.getRadians();

        m_pid.enableContinuousInput(-Math.PI, Math.PI);
        m_pid.setTolerance(0.05);

        addRequirements(m_drive);

    }

    @Override
    public void initialize() {
        m_finished = false;
        m_pid.reset(m_drive.getGyro().getRadians(), m_drive.getChassisSpeed().omegaRadiansPerSecond);
        m_pid.setGoal(m_angle);
    }

    @Override
    public void execute() {

        double pid = m_pid.calculate(m_drive.getGyro().getRadians(), m_angle);

        TrapezoidProfile.State state = m_pid.getSetpoint();

        double desiredRot = pid + state.velocity;

        if (m_pid.atGoal()) {
            m_finished = true;
        }

        m_drive.drive(0.0, 0.0, desiredRot, true, true);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }

}
