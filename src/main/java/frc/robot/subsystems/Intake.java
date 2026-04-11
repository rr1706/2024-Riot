package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;

public class Intake extends SubsystemBase {
    private final SparkMax m_kick = new SparkMax(10, MotorType.kBrushless);
    private final SparkMaxConfig m_kickConfig = new SparkMaxConfig();
    private final TalonFX m_intake = new TalonFX(11,"rio");
    private Slot0Configs slot0Configs = new Slot0Configs();


    public Intake() {
        m_kickConfig.smartCurrentLimit(CurrentLimit.kKicker);
        m_kickConfig.voltageCompensation(GlobalConstants.kVoltCompensation);
        m_kickConfig.idleMode(IdleMode.kBrake);
        m_kickConfig.inverted(false);

        m_kick.configure(m_kickConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        m_intake.getConfigurator().apply(slot0Configs);
        m_intake.getConfigurator().apply(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(CurrentLimit.kIntakerStator)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(CurrentLimit.kIntakerSupply)
                .withSupplyCurrentLimitEnable(true));
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

    public double getKickerCurrent() {
        return m_kick.getOutputCurrent();
    }

    public void stop() {
        m_kick.stopMotor();
        m_intake.stopMotor();
    }
}
