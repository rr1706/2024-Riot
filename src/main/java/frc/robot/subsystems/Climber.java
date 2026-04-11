package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;

public class Climber extends SubsystemBase {
    private final SparkMax m_climberL = new SparkMax(13, MotorType.kBrushless);
    private final SparkMax m_climberR = new SparkMax(12, MotorType.kBrushless);
    private final SparkClosedLoopController m_pidL = m_climberL.getClosedLoopController();
    private final SparkClosedLoopController m_pidR = m_climberR.getClosedLoopController();
    private final RelativeEncoder m_leftEncoder = m_climberL.getEncoder();
    private final RelativeEncoder m_rightEncoder = m_climberR.getEncoder();
    private final SparkMaxConfig m_configL = new SparkMaxConfig();
    private final SparkMaxConfig m_configR = new SparkMaxConfig();

    private double m_leftPose = 2.0;
    private double m_rightPose = 2.0;

    private boolean m_leftPIDEnabled = true;
    private boolean m_rightPIDEnabled = true;

    public Climber() {
        m_configL.smartCurrentLimit(CurrentLimit.kClimber);
        m_configL.voltageCompensation(GlobalConstants.kVoltCompensation);
        m_configL.idleMode(IdleMode.kBrake);
        m_configL.closedLoop.p(1.0);
        m_configL.inverted(true);
        m_climberL.configure(m_configL, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        m_configR.smartCurrentLimit(CurrentLimit.kClimber);
        m_configR.voltageCompensation(GlobalConstants.kVoltCompensation);
        m_configR.idleMode(IdleMode.kBrake);
        m_configR.closedLoop.p(1.0);
        m_configR.inverted(false);
        m_climberR.configure(m_configR, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void zero() {
        m_leftPIDEnabled = false;
        m_rightPIDEnabled = false;
        m_climberL.set(-0.2);
        m_climberR.set(-0.2);
    }

    public void setPose(double pose) {
        m_leftPIDEnabled = true;
        m_rightPIDEnabled = true;
        m_leftPose = pose;
        m_rightPose = pose;
    }

    public void setPose(double left, double right) {
        m_leftPIDEnabled = true;
        m_rightPIDEnabled = true;
        m_leftPose = left;
        m_rightPose = right;
    }

    public void setLeftZero() {
        m_leftEncoder.setPosition(0.0);
    }

    public void setRightZero() {
        m_rightEncoder.setPosition(0.0);
    }

    public double getLeftCurrent() {
        return m_climberL.getOutputCurrent();
    }

    public double getRightCurrent() {
        return m_climberR.getOutputCurrent();
    }

    public void stopLeft() {
        m_climberL.stopMotor();
    }

    public void stopRight() {
        m_climberR.stopMotor();
    }

    public double getLeftPose() {
        return m_leftEncoder.getPosition();
    }

    public double getRightPose() {
        return m_rightEncoder.getPosition();
    }

    @Override
    public void periodic() {
        if (m_leftPIDEnabled) {
            m_pidL.setSetpoint(m_leftPose, ControlType.kPosition);
        }
        if (m_rightPIDEnabled) {
            m_pidR.setSetpoint(m_rightPose, ControlType.kPosition);
        }

        SmartDashboard.putNumber("Left Climber", getLeftPose());
        SmartDashboard.putNumber("Right Climber", getRightPose());
    }

}
