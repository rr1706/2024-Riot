package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.PitcherConstants;

public class Pitcher extends SubsystemBase {
    private final CANSparkMax m_motor = new CANSparkMax(5, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkPIDController m_pid = m_motor.getPIDController();
    private double m_angle = 5.0;
    private boolean m_PIDEnabled = true;

    public Pitcher() {
        m_motor.setSmartCurrentLimit(CurrentLimit.kPitcher);
        m_motor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_motor.setIdleMode(IdleMode.kBrake);
        m_pid.setP(PitcherConstants.kP);

        m_motor.setInverted(true);
        m_motor.burnFlash();
    }

    public void pitchToAngle(double angle) {
        m_PIDEnabled = true;
        if (angle >= 20.0) {
            angle = 20.0;
        } else if (angle <= 2.0) {
            angle = 2.0;
        }
        m_angle = angle;
    }

    public Command changePitch(double adjust) {
        return runOnce(() -> {
            m_PIDEnabled = true;
            m_angle += adjust;
            if (m_angle >= 20.0) {
                m_angle = 20.0;
            } else if (m_angle <= 2.0) {
                m_angle = 2.0;
            }
        });
    }

    public void stop() {
        m_motor.stopMotor();
    }

    public void setZero() {
        m_encoder.setPosition(-0.4);
    }

    public void zero() {
        m_PIDEnabled = false;
        m_motor.set(-0.1);
    }

    public double getCurrent() {
        return m_motor.getOutputCurrent();
    }

    @Override
    public void periodic() {
        if (m_PIDEnabled) {
            m_pid.setReference(m_angle, ControlType.kPosition);
        }

        SmartDashboard.putNumber("Pitcher", m_motor.getEncoder().getPosition());
    }

}
