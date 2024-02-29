package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;

public class Feeder extends SubsystemBase{

    private final CANSparkMax m_motor = new CANSparkMax(6, MotorType.kBrushless);

    public Feeder(){

        m_motor.setSmartCurrentLimit(CurrentLimit.kFeeder);
        m_motor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_motor.setIdleMode(IdleMode.kBrake);

        m_motor.setInverted(false);
        m_motor.burnFlash();
    }

    public void run(double speed){
        m_motor.set(speed);
    }
    public void stop(){
        m_motor.stopMotor();
    }

}
