package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;

public class Intake extends SubsystemBase {
    private final CANSparkMax m_kick = new CANSparkMax(10, MotorType.kBrushless);
    private final TalonFX m_intake = new TalonFX(11);
    private Slot0Configs slot0Configs = new Slot0Configs();

    public Intake() {
        m_kick.setSmartCurrentLimit(CurrentLimit.kKicker);
        m_kick.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_kick.setIdleMode(IdleMode.kBrake);

        m_kick.setInverted(false);
        m_kick.burnFlash();

        m_intake.getConfigurator().apply(slot0Configs);
        m_intake.getConfigurator().apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(CurrentLimit.kIntakerStator)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(CurrentLimit.kIntakerSupply)
            .withSupplyCurrentLimitEnable(true));
        m_intake.setInverted(false);
        m_intake.setNeutralMode(NeutralModeValue.Brake);


    }

    public void run(double speed) {
        m_kick.set(speed * 0.7);
        m_intake.set(speed);
    }

    public void runIndividual(double kick, double intake) {
        m_kick.set(kick);
        m_intake.set(intake);
    }

    public void run(double speed, double chassisVelocity) {
        if (chassisVelocity <= 0) {
            m_kick.set(speed * 0.7);
            m_intake.set(speed);
        } else if (chassisVelocity > 0) {
            m_kick.set(-speed * 0.7);
            m_intake.set(speed);
        }

    }

    public double getKickerCurrent(){
        return m_kick.getOutputCurrent();
    }

    public void stop() {
        m_kick.stopMotor();
        m_intake.stopMotor();
    }
}
