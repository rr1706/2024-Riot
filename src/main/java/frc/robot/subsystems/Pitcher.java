package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.PitcherConstants;

public class Pitcher extends SubsystemBase{
    private final CANSparkMax m_motor = new CANSparkMax(5,MotorType.kBrushless);
    private final SparkPIDController m_pid = m_motor.getPIDController();
    private double m_angle = 3.0;

    public Pitcher(){
        m_motor.setSmartCurrentLimit(CurrentLimit.kPitcher);
        m_motor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_motor.setIdleMode(IdleMode.kBrake);
        m_pid.setP(PitcherConstants.kP);

        m_motor.setInverted(true);
        m_motor.burnFlash();
    }

    public void pitchToAngle(double angle){
        m_angle = angle;
    }

    @Override
    public void periodic() {
        m_pid.setReference(m_angle, ControlType.kPosition);
    }

}
