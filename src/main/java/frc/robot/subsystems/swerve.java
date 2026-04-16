package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.ModuleConstants.Aziumth;
import frc.robot.Constants.ModuleConstants.Drive;

public class swerve extends SubsystemBase {
    private final SparkMax m_azimuth;
    private final TalonFX m_drive;

    private final SparkMaxConfig m_azConfig = new SparkMaxConfig();
    private final TalonFXConfigurator m_drConfig;

    private final SparkClosedLoopController m_azPID;
    private final VelocityVoltage m_req = new VelocityVoltage(0.0);

    private final double m_offset;
    private final SparkAbsoluteEncoder m_encoder;

    private final PIDController m_azPIDController = new PIDController(1.0, 0.0, 0.0);

    public swerve(int module, double offset) {
        m_azimuth = new SparkMax(module, MotorType.kBrushless);
        m_azPID = m_azimuth.getClosedLoopController();
        m_azConfig.smartCurrentLimit(CurrentLimit.kAzimuth)
                    .voltageCompensation(GlobalConstants.kVoltCompensation)
                    .idleMode(IdleMode.kBrake);

        m_azConfig.inverted(true);
        m_azConfig.idleMode(IdleMode.kBrake);

        m_azConfig.encoder.positionConversionFactor(Aziumth.kPositionFactor);
        m_azConfig.encoder.velocityConversionFactor(Aziumth.kVelocityFactor);

        m_azConfig.closedLoop.p(Aziumth.kp).feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        m_azimuth.configure(m_azConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        m_encoder = m_azimuth.getAbsoluteEncoder();

        m_offset = offset;

        m_azPIDController.enableContinuousInput(-Math.PI, Math.PI);

        m_drive = new TalonFX(module, new CANBus("rio"));
        m_drConfig = m_drive.getConfigurator();

        m_drConfig.apply(new Slot0Configs()
                            .withKP(0.05)
                            .withKS(0.05)
                            .withKV(0.12));

        m_drConfig.apply(new ClosedLoopRampsConfigs()
                            .withVoltageClosedLoopRampPeriod(0.100));

        m_drConfig.apply(new CurrentLimitsConfigs()
                            .withSupplyCurrentLimit(CurrentLimit.kDriveSupply)
                            .withStatorCurrentLimit(CurrentLimit.kDriveStator)
                            .withSupplyCurrentLimitEnable(true)
                            .withStatorCurrentLimitEnable(true));

        m_drive.setNeutralMode(NeutralModeValue.Brake);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getStateAngle()));
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDriveVelocity(), new Rotation2d(getStateAngle()));
    }

    public double getDriveVelocity(){
        return m_drive.getVelocity().getValueAsDouble() * Drive.kToMeters;
    }

    public double getDrivePosition(){
        return m_drive.getPosition().getValueAsDouble() * Drive.kToMeters;
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getStateAngle()));
        double metersToRotations = state.speedMetersPerSecond * Drive.kToRots;
        m_drive.setControl(m_req.withVelocity(metersToRotations));
        m_azPID.setSetpoint(state.angle.getRadians(), ControlType.kPosition);
    }

    public double getStateAngle() {
        return m_encoder.getPosition();
    }

    public void stop() {
        m_drive.stopMotor();
        m_azimuth.stopMotor();
    }

    public Command stopCmd() {
        return runOnce(()-> stop());
    }
}
