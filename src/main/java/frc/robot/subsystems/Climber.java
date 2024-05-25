package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;

public class Climber extends SubsystemBase {
    private final CANSparkMax m_climberL = new CANSparkMax(13, MotorType.kBrushless);
    private final CANSparkMax m_climberR = new CANSparkMax(12, MotorType.kBrushless);
    private final SparkPIDController m_pidL = m_climberL.getPIDController();
    private final SparkPIDController m_pidR = m_climberR.getPIDController();
    private final RelativeEncoder m_leftEncoder = m_climberL.getEncoder();
    private final RelativeEncoder m_rightEncoder = m_climberR.getEncoder();
    private double m_leftPose = 2.0;
    private double m_rightPose = 2.0;

    private boolean m_leftPIDEnabled = true;
    private boolean m_rightPIDEnabled = true;

    public Climber() {
        m_climberL.setSmartCurrentLimit(CurrentLimit.kClimber);
        m_climberL.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_climberL.setIdleMode(IdleMode.kBrake);
        m_pidL.setP(0.25);

        m_climberL.setInverted(true);
        m_climberL.burnFlash();

        m_climberR.setSmartCurrentLimit(CurrentLimit.kClimber);
        m_climberR.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_climberR.setIdleMode(IdleMode.kBrake);
        m_pidR.setP(0.25);

        m_climberR.setInverted(false);
        m_climberR.burnFlash();
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
            m_pidL.setReference(m_leftPose, ControlType.kPosition);
        }
        if (m_rightPIDEnabled) {
            m_pidR.setReference(m_rightPose, ControlType.kPosition);
        }
        SmartDashboard.putNumber("Left Climber", getLeftPose());
        SmartDashboard.putNumber("Right Climber", getRightPose());

    }

}
