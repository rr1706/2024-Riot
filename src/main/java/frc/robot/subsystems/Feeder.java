package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;

public class Feeder extends SubsystemBase {
    private boolean m_PIDEnabled = false;
    private double m_desiredPose = 0.0;
    private final DigitalInput m_prox = new DigitalInput(3);
    private final TalonFX m_motor = new TalonFX(7,"*");
    private Slot0Configs slot0Configs = new Slot0Configs();

    public Feeder() {
        configurePID();
        m_motor.getConfigurator().apply(slot0Configs);
        m_motor.getConfigurator().apply(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(CurrentLimit.kFeederStator)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(CurrentLimit.kFeederSupply)
                .withSupplyCurrentLimitEnable(true));
        m_motor.setNeutralMode(NeutralModeValue.Brake);

    }

    @Override
    public void periodic() {
        if (m_PIDEnabled) {
            m_motor.setControl(new MotionMagicVoltage(m_desiredPose));
        }
        SmartDashboard.putBoolean("Feeder Prox", getProx());
    }

    public void configurePID() {
        slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.01; // An error of 1 rps results in 0.01 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative
    }

    public void run(double speed) {
        m_PIDEnabled = false;
        m_motor.setControl(new DutyCycleOut(speed));
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

    public double getEncoder() {
        return m_motor.getPosition().getValueAsDouble();
    }

    public boolean atSetpoint() {
        return (Math.abs(m_desiredPose - getEncoder())) <= 0.5;
    }

    public void setPose(double pose) {
        m_desiredPose = pose;
        m_PIDEnabled = true;
    }

    public double getCurrent() {
        return m_motor.getStatorCurrent().getValueAsDouble();
    }
}
