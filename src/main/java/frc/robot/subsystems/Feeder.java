package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;

public class Feeder extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(7,"rio");
    private final TalonFXConfigurator m_cfg = m_motor.getConfigurator();

    private final DigitalInput m_prox = new DigitalInput(3);

    private boolean m_PIDEnabled = false;
    private double m_desiredPose = 0.0;

    public Feeder() {
        motorConfigs();
    }

    public void run(double speed) {
        m_PIDEnabled = false;
        m_motor.setControl(new DutyCycleOut(speed));
    }

    public Command runCmd(double speed) {
        return runEnd(()-> run(speed), ()-> stop());
    }

    public void stop() {
        m_PIDEnabled = false;
        m_motor.stopMotor();
    }

    public boolean getProx(){
        return !m_prox.get();
    }

    public void setZero() {
        m_motor.setPosition(0.0);
    }

    public double getPosition() {
        return m_motor.getPosition().getValueAsDouble();
    }

    public boolean atSetpoint() {
        return (Math.abs(m_desiredPose - getPosition())) <= 0.5;
    }

    public void setPose(double pose) {
        m_desiredPose = pose;
        m_PIDEnabled = true;
    }

    public double getCurrent() {
        return m_motor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void periodic() {
        if (m_PIDEnabled) m_motor.setControl(new MotionMagicVoltage(m_desiredPose));

        SmartDashboard.putBoolean("Feeder Prox", getProx());
    }

    public void motorConfigs() {
        m_cfg.apply(new Slot0Configs()
                .withKS(0.05).withKV(0.12).withKP(0.01));

        m_cfg.apply(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(CurrentLimit.kFeederStator)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(CurrentLimit.kFeederSupply)
                .withSupplyCurrentLimitEnable(true));

        m_motor.setNeutralMode(NeutralModeValue.Brake);
    }
}
