package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;

public class Shooter extends SubsystemBase {
    private final TalonFX m_motor1 = new TalonFX(5,"rio");
    private final TalonFX m_motor2 = new TalonFX(6,"rio");

    private double m_desiredSpin = 0.0;
    private double m_desriedVel = 0.0;

    private final VelocityVoltage m_request = 
            new VelocityVoltage(0.0).withSlot(0);

    public Shooter() {
        motorConfigs(m_motor1);
        motorConfigs(m_motor2);
    }

    public void run(double velocity) {
        m_desriedVel = velocity;

        m_motor1.setControl(m_request.withVelocity(velocity));
        m_motor2.setControl(m_request.withVelocity(-1.0 * velocity));
    }

    public Command changeSpeed(double adjust) {
        m_desriedVel += adjust;

        return runOnce(() -> {
            if (m_desriedVel >= 80.0) m_desriedVel = 80.0;
            else if (m_desriedVel <= 10.0) m_desriedVel = 10.0;
        });
    }

    public void run(double velocity, double spinDiff) {
        spinDiff = 0.01 * spinDiff * velocity;

        if(velocity >= 100.0) velocity = 100.0;
        else if (velocity <= -10.0) velocity = -10.0;

        m_desriedVel = velocity;
        m_desiredSpin = spinDiff;
    }

    public Command runShooter(double velocity, double spinDiff){
        return runEnd(()-> run(velocity, spinDiff), ()-> stop());
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

    public Command stopCmd() {
        return runOnce(()-> stop());
    }

    public boolean atSetpoint() {
        return Math.abs(m_motor1.getVelocity().getValueAsDouble() - m_desriedVel) <= 5.0;
    }

    @Override
    public void periodic() {
        m_motor1.setControl(m_request.withVelocity(m_desriedVel + m_desiredSpin / 2.0));
        m_motor2.setControl(m_request.withVelocity(-1.0 * (m_desriedVel - m_desiredSpin / 2.0)));
    }

    public void motorConfigs(TalonFX motor) {
        TalonFXConfigurator config = motor.getConfigurator();

        motor.setNeutralMode(NeutralModeValue.Brake);

        config.apply(new Slot0Configs()
                .withKS(0.05).withKV(0.12).withKP(0.10));

        config.apply(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(CurrentLimit.kShooterStator)
                .withSupplyCurrentLimit(CurrentLimit.kShooterSupply)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true));
    }
}
