package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;

public class Feeder extends SubsystemBase{
    private final CANSparkMax m_motor = new CANSparkMax(6, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkPIDController m_pid = m_motor.getPIDController();
    private boolean m_PIDEnabled = false;
    private double m_desiredPose = 0.0;

    public Feeder(){
        m_motor.setSmartCurrentLimit(CurrentLimit.kFeeder);
        m_motor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_motor.setIdleMode(IdleMode.kBrake);
        m_pid.setP(0.175);

        m_motor.setInverted(false);
        m_motor.burnFlash();
    }

    @Override
    public void periodic() {
        if(m_PIDEnabled){
            m_pid.setReference(m_desiredPose, ControlType.kPosition);
        }
        SmartDashboard.putNumber("Manipulator Pose", getEncoder());
        SmartDashboard.putNumber("Feeder Current", getCurrent());
        SmartDashboard.putNumber("Feeder Power", getPower());
    }

    public void run(double speed){
        m_PIDEnabled = false;
        m_motor.set(speed);
    }

    public void stop(){
        m_PIDEnabled = false;
        m_motor.stopMotor();
    }

    public void setZero(){
        m_encoder.setPosition(0.0);
    }

    public double getEncoder(){
        return m_encoder.getPosition();
    }

    public boolean atSetpoint(){
        return (Math.abs(m_desiredPose - getEncoder())) <= 0.5;
    }

    public void setPose(double pose){
        m_desiredPose = pose;
        m_PIDEnabled = true;
    }

    public double getCurrent(){
        return m_motor.getOutputCurrent();
    }

        public double getPower(){
        return m_motor.getOutputCurrent()*m_motor.getAppliedOutput()*m_motor.getBusVoltage();
    }
}
