package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;

public class Shooter extends SubsystemBase {

    // private final CANSparkMax m_motor1 = new CANSparkMax(7,
    // MotorType.kBrushless);
    // private final CANSparkMax m_motor2 = new CANSparkMax(8,
    // MotorType.kBrushless);

    private final TalonFX m_motor1 = new TalonFX(5);
    private final TalonFX m_motor2 = new TalonFX(6);

    private double m_desriedVel = 0.0;

    private Slot0Configs slot0Configs = new Slot0Configs();
    private final VelocityVoltage m_request = new VelocityVoltage(0.0).withSlot(0);

    // private final RelativeEncoder m_encoder1 = m_motor1.getEncoder();
    // private final SparkPIDController m_pid = m_motor1.getPIDController();

    public Shooter() {
        // m_motor1.restoreFactoryDefaults(false);
        // m_motor2.restoreFactoryDefaults(false);
        // m_motor1.setSmartCurrentLimit(CurrentLimit.kShooter);
        // m_motor2.setSmartCurrentLimit(CurrentLimit.kShooter);
        // m_motor1.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        // m_motor2.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        // m_motor1.setIdleMode(IdleMode.kCoast);
        // m_motor2.setIdleMode(IdleMode.kCoast);

        // m_pid.setFF(1/5676.0);
        // m_pid.setP(0.0001);

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
        slot0Configs.kP = 0.01; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter", m_motor1.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Motor1", m_motor1.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Motor2", m_motor2.getStatorCurrent().getValueAsDouble());

    }

    public void run(double velocity) {
        m_desriedVel = velocity;
        m_motor1.setControl(m_request.withVelocity(velocity).withSlot(0));
        m_motor2.setControl(m_request.withVelocity(-1.0 * velocity).withSlot(0));

    }

    public void run(double velocity, double spinDiff) {
        m_desriedVel = velocity;
        spinDiff = 0.01*spinDiff*velocity;
        m_motor1.setControl(m_request.withVelocity(velocity+spinDiff/2.0).withSlot(0));
        m_motor2.setControl(m_request.withVelocity(-1.0 * (velocity-spinDiff/2.0)).withSlot(0));

    }

    public void stop() {
        m_motor1.stopMotor();
        m_motor2.stopMotor();
    }

    public boolean atSetpoint() {
        return Math.abs(m_motor1.getVelocity().getValueAsDouble() - m_desriedVel) <= 5.0;
    }

}
