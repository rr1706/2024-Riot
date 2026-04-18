package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import org.ejml.dense.row.mult.MatrixMatrixMult_MT_ZDRM;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
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

/**
 * FRC 1706 Class for creating a swerve module with 2 SparkMax motor controllers
 * and an analog azimuth Encoder
 */
public class swerve extends SubsystemBase {
    private final SparkMax m_azimuthMotor;
    private final SparkMaxConfig m_azConfig = new SparkMaxConfig();
    private final TalonFX m_driveMotor;
    private final AbsoluteEncoder m_azimuthEnc;
    private final SparkClosedLoopController m_azimuthPID;
    private Slot0Configs slot0Configs = new Slot0Configs();
    private final VelocityVoltage m_request = new VelocityVoltage(0.0).withSlot(0);
    private final PIDController m_aziPID = new PIDController(1.0, 0.0, 0.0);

    /**
     * Create a new FRC 1706 NEOKrakenSwerveModule Object
     *
     * @param moduleID  module ID also CAN ID for Azimuth and Drive MCs.
     * @param offset  The offset for the analog encoder.
     */
    public swerve(int moduleID, double offset) {
        m_aziPID.enableContinuousInput(-Math.PI, Math.PI);

        configurePID();
        m_driveMotor = new TalonFX(moduleID,"rio");
        m_driveMotor.getConfigurator().apply(slot0Configs);
        m_driveMotor.getConfigurator()
                .apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(CurrentLimit.kDriveSupply)
                        .withStatorCurrentLimit(CurrentLimit.kDriveStator)
                        .withSupplyCurrentLimitEnable(true)
                        .withStatorCurrentLimitEnable(true));
        m_driveMotor.setNeutralMode(NeutralModeValue.Brake);

        m_driveMotor.getConfigurator().apply(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.100));

        m_azimuthMotor = new SparkMax(moduleID, MotorType.kBrushless);

        m_azConfig.smartCurrentLimit(CurrentLimit.kAzimuth);
        m_azConfig.voltageCompensation(GlobalConstants.kVoltCompensation);
        m_azConfig.inverted(false);
        m_azConfig.idleMode(IdleMode.kBrake);

        m_azConfig.encoder.positionConversionFactor(Aziumth.kPositionFactor);
        m_azConfig.encoder.velocityConversionFactor(Aziumth.kVelocityFactor);
        //m_azConfig.encoder.inverted(true);

        m_azimuthEnc = m_azimuthMotor.getAbsoluteEncoder();

        m_azimuthPID = m_azimuthMotor.getClosedLoopController();

        //m_azConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        // m_azConfig.closedLoop.positionWrappingEnabled(true);
        // m_azConfig.closedLoop.positionWrappingMinInput(0.0);
        // m_azConfig.closedLoop.positionWrappingMaxInput(2.0 * Math.PI);

        //m_azConfig.closedLoop.p(Aziumth.kp);

        m_azimuthMotor.configure(m_azConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getStateAngle()));
    }

    /**
     * 
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getStateAngle()));
    }

    public void configurePID() {
        slot0Configs.kS = 0.1; // Add 0.05 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.05; // An error of 1 rps results in 0.05 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative
    }

    public double getDriveVelocity() {
        return m_driveMotor.getVelocity().getValueAsDouble() * Drive.kToMeters;
    }

    public double getDrivePosition() {
        return m_driveMotor.getPosition().getValueAsDouble() * Drive.kToMeters;
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getStateAngle()));
        double metersToRotations = state.speedMetersPerSecond * Drive.kToRots;
        m_driveMotor.setControl(m_request.withVelocity(metersToRotations).withSlot(0));
        //m_azimuthPID.setSetpoint(state.angle.getRadians(), ControlType.kPosition);
        double pidOutput = m_aziPID.calculate(getStateAngle(), desiredState.angle.getRadians());
        m_azimuthMotor.set(pidOutput);
    }

    public double getStateAngle() {
        return m_azimuthEnc.getPosition()*(2*Math.PI);
    }

    public void stop() {
        m_driveMotor.stopMotor();
        m_azimuthMotor.stopMotor();
    }
}