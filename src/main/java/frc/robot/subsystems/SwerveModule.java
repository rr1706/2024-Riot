package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.ModuleConstants.Aziumth;
import frc.robot.Constants.ModuleConstants.Drive;

public class SwerveModule extends SubsystemBase {
    private final SparkMax m_azimuthMotor;
    private final TalonFX m_driveMotor;

    private final SparkMaxConfig m_azConfig = new SparkMaxConfig();
    private final TalonFXConfigurator m_drConfig;

    private final AbsoluteEncoder m_encoder;

    private final VelocityVoltage m_request = new VelocityVoltage(0.0);
    private final PIDController m_aziPID = new PIDController(0.5, 0.0, 0.0);

    public SwerveModule(int moduleID) {
        m_driveMotor = new TalonFX(moduleID,"rio");
        m_azimuthMotor = new SparkMax(moduleID, MotorType.kBrushless);

        m_drConfig = m_driveMotor.getConfigurator();
        m_encoder = m_azimuthMotor.getAbsoluteEncoder();

        driveConfigs();
        azimuthConfigs();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getStateAngle()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getStateAngle()));
    }

    public double getDriveVelocity() {
        return m_driveMotor.getVelocity().getValueAsDouble() * Drive.kToMeters;
    }

    public double getDrivePosition() {
        return m_driveMotor.getPosition().getValueAsDouble() * Drive.kToMeters;
    }

    public double getStateAngle() {
        return m_encoder.getPosition()*(2*Math.PI);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState newState = new SwerveModuleState();

        newState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        newState.angle = desiredState.angle;

        newState.optimize(new Rotation2d(getStateAngle()));

        m_driveMotor.setControl(m_request.withVelocity(newState.speedMetersPerSecond*Drive.kToRots));
        m_azimuthMotor.set(m_aziPID.calculate(getStateAngle(), newState.angle.getRadians()));
    }

    public void stop() {
        m_driveMotor.stopMotor();
        m_azimuthMotor.stopMotor();
    }

    public void driveConfigs() {
        m_drConfig.apply(new Slot0Configs()
                .withKS(0.05).withKV(0.12).withKP(0.05));

        m_drConfig.apply(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(CurrentLimit.kDriveSupply)
                .withStatorCurrentLimit(CurrentLimit.kDriveStator)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimitEnable(true));

        m_drConfig.apply(new ClosedLoopRampsConfigs()
                .withVoltageClosedLoopRampPeriod(0.100));

        m_driveMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void azimuthConfigs() {
        m_azConfig.smartCurrentLimit(CurrentLimit.kAzimuth);
        m_azConfig.voltageCompensation(GlobalConstants.kVoltCompensation);

        m_azConfig.inverted(true);
        m_azConfig.idleMode(IdleMode.kBrake);

        m_azConfig.encoder.positionConversionFactor(Aziumth.kPositionFactor);
        m_azConfig.encoder.velocityConversionFactor(Aziumth.kVelocityFactor);

        m_azConfig.absoluteEncoder.inverted(true);

        m_azConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        m_azConfig.closedLoop.positionWrappingEnabled(true);
        m_azConfig.closedLoop.positionWrappingMinInput(0.0);
        m_azConfig.closedLoop.positionWrappingMaxInput(2.0 * Math.PI);

        m_azConfig.closedLoop.p(Aziumth.kp);

        m_azimuthMotor.configure(m_azConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        m_aziPID.enableContinuousInput(-Math.PI, Math.PI);
    }
}