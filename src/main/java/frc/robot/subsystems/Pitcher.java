package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.PitcherConstants;

public class Pitcher extends SubsystemBase {
    private final SparkMax m_motor = new SparkMax(5, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkClosedLoopController m_pid = m_motor.getClosedLoopController();
    private final SparkMaxConfig m_config = new SparkMaxConfig();
    private double m_angle = 5.0;
    private boolean m_PIDEnabled = true;

    public Pitcher() {
        m_config.smartCurrentLimit(CurrentLimit.kPitcher);
        m_config.voltageCompensation(GlobalConstants.kVoltCompensation);
        m_config.idleMode(IdleMode.kBrake);
        m_config.closedLoop.p(PitcherConstants.kP);
        m_config.inverted(true);

        m_motor.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
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

    public double getPitch(){
        return m_motor.getEncoder().getPosition();
    }

    public double getSetAngle(){
        return m_angle;
    }

    @Override
    public void periodic() {
        if (m_PIDEnabled) {
            m_pid.setSetpoint(m_angle, ControlType.kPosition);
        }

        SmartDashboard.putNumber("Pitcher", getPitch());
    }

}
