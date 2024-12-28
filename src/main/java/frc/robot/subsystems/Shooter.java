package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;

public class Shooter extends SubsystemBase {

    private final TalonFX m_motor1 = new TalonFX(5,"*");
    private final TalonFX m_motor2 = new TalonFX(6,"*");

    private double m_desiredSpin = 0.0;

    private double m_desriedVel = 0.0;

    private Slot0Configs slot0Configs = new Slot0Configs();
    private final VelocityVoltage m_request = new VelocityVoltage(0.0).withSlot(0);

    public Shooter() {

        configurePID();
        m_motor1.getConfigurator().apply(slot0Configs);
        m_motor2.getConfigurator().apply(slot0Configs);
        m_motor1.getConfigurator().apply(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(CurrentLimit.kShooterStator)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(CurrentLimit.kShooterSupply)
                .withSupplyCurrentLimitEnable(true));
        m_motor2.getConfigurator().apply(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(CurrentLimit.kShooterStator)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(CurrentLimit.kShooterSupply)
                .withSupplyCurrentLimitEnable(true));

        m_motor1.setNeutralMode(NeutralModeValue.Brake);
        m_motor2.setNeutralMode(NeutralModeValue.Brake);

    }

    public void configurePID() {
        slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.10; // An error of 1 rps results in 0.10 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative
    }

    @Override
    public void periodic() {
        m_motor1.setControl(m_request.withVelocity(m_desriedVel + m_desiredSpin / 2.0).withSlot(0));
        m_motor2.setControl(m_request.withVelocity(-1.0 * (m_desriedVel - m_desiredSpin / 2.0)).withSlot(0));

    }

    public void run(double velocity) {
        m_desriedVel = velocity;
        m_motor1.setControl(m_request.withVelocity(velocity).withSlot(0));
        m_motor2.setControl(m_request.withVelocity(-1.0 * velocity).withSlot(0));

    }

    public Command changeSpeed(double adjust) {
        return runOnce(() -> {
            m_desriedVel += adjust;
            if (m_desriedVel >= 80.0) {
                m_desriedVel = 80.0;
            } else if (m_desriedVel <= 10.0) {
                m_desriedVel = 10.0;
            }
        });
    }

    public void run(double velocity, double spinDiff) {
        spinDiff = 0.01 * spinDiff * velocity;
        if(velocity >= 100.0){
            velocity = 100.0;
        }
        else if (velocity <= -10.0){
            velocity = -10.0;
        }
        m_desriedVel = velocity;
        m_desiredSpin = spinDiff;
    }

    public Command runShooter(double velocity, double spinDiff){
        return runEnd(()->run(velocity, spinDiff), ()->stop());
    }

    public double getSetVelocity(){
        return m_desriedVel;    
    }

    public void stop() {
        m_motor1.stopMotor();
        m_motor2.stopMotor();
        m_desriedVel = 0.0;
        m_desiredSpin = 0.0;
    }

    public boolean atSetpoint() {
        return Math.abs(m_motor1.getVelocity().getValueAsDouble() - m_desriedVel) <= 5.0;
    }

}
