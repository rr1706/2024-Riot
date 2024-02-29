package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;

public class Intake extends SubsystemBase{
    private final CANSparkMax m_kick = new CANSparkMax(10, MotorType.kBrushless);
    private final CANSparkMax m_intake = new CANSparkMax(11, MotorType.kBrushless);
    
    public Intake(){
        m_kick.setSmartCurrentLimit(CurrentLimit.kKicker);
        m_kick.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_kick.setIdleMode(IdleMode.kBrake);

        m_kick.setInverted(false);
        m_kick.burnFlash();

        m_intake.setSmartCurrentLimit(CurrentLimit.kIntaker);
        m_intake.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_intake.setIdleMode(IdleMode.kBrake);

        m_intake.setInverted(false);
        m_intake.burnFlash();
    }

    public void run(double speed){
        m_kick.set(speed*0.6667);
        m_intake.set(speed);
    }
    public void run(double speed, double chassisVelocity){
        if(chassisVelocity <= 0){
            m_kick.set(speed*0.6667);
            m_intake.set(speed);
        }
        else if (chassisVelocity > 0){
            m_kick.set(-speed*0.6667);
            m_intake.set(speed);
        }
        
    }
    public void stop(){
        m_kick.stopMotor();
        m_intake.stopMotor();
    }
}
