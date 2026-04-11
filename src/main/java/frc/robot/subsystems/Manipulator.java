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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;

public class Manipulator extends SubsystemBase {
    private final SparkMax m_motor = new SparkMax(16, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkClosedLoopController m_pid = m_motor.getClosedLoopController();
    private final SparkMaxConfig m_config = new SparkMaxConfig();

    private boolean m_PIDEnabled = false;
    private double m_desiredPose = 0.0;

    public Manipulator() {
        m_config.smartCurrentLimit(CurrentLimit.kManipulator);
        m_config.voltageCompensation(GlobalConstants.kVoltCompensation);
        m_config.idleMode(IdleMode.kBrake);
        m_config.closedLoop.p(0.175);
        m_config.inverted(false);

        m_motor.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        if (m_PIDEnabled) {
            m_pid.setSetpoint(m_desiredPose, ControlType.kPosition);
        }
    }

    public void run(double speed) {
        m_PIDEnabled = false;
        m_motor.set(speed);
    }

    public void stop() {
        m_PIDEnabled = false;
        m_motor.stopMotor();
    }

    public void setZero() {
        m_encoder.setPosition(0.0);
    }

    public double getEncoder() {
        return m_encoder.getPosition();
    }

    public boolean atSetpoint() {
        return (Math.abs(m_desiredPose - getEncoder())) <= 0.5;
    }

    public void setPose(double pose) {
        m_desiredPose = pose;
        m_PIDEnabled = true;
    }

    public double getCurrent() {
        return m_motor.getOutputCurrent();
    }
}
