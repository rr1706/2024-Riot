package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;

public class Indexer extends SubsystemBase {
    private final SparkMax m_motor = new SparkMax(9, MotorType.kBrushless);
    private final SparkMaxConfig m_config = new SparkMaxConfig();

    public Indexer() {
        m_config.smartCurrentLimit(CurrentLimit.kIndexer);
        m_config.voltageCompensation(GlobalConstants.kVoltCompensation);
        m_config.idleMode(IdleMode.kBrake);
        m_config.inverted(false);

        m_motor.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void run(double speed) {
        m_motor.set(speed);
    }

    public void stop() {
        m_motor.stopMotor();
    }
}
