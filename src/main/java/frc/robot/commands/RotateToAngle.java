package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class RotateToAngle extends Command {

    private final Drivetrain m_drive;
    private final double m_angle;
    private final PIDController m_pid = new PIDController(0.12, 0.00, 0.0);
    private boolean m_finished = false;

    public RotateToAngle(Rotation2d angle, Drivetrain drive){
        m_drive=drive;
        m_angle = angle.getDegrees();

        m_pid.enableContinuousInput(-180.0, 180.0);
        m_pid.setTolerance(10.0);

        addRequirements(m_drive);

    }

    @Override
    public void initialize() {
        m_finished = false;
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Rotate Done", m_pid.atSetpoint());
        if(m_pid.atSetpoint()){
            m_finished = true;
        }
        double desiredRot = m_pid.calculate(m_drive.getGyro().getDegrees(),m_angle);

        if(desiredRot>=1.2*Math.PI){
            desiredRot = 1.2*Math.PI;
        }
        else if(desiredRot<=-1.2*Math.PI){
                        desiredRot = -1.2*Math.PI;

        }
        m_drive.drive(0.0, 0.0, desiredRot, true, false);
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
