package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
public class NEOKrakenSwerveModule extends SubsystemBase {
    private final CANSparkMax m_azimuthMotor;
    private final TalonFX m_driveMotor;
    private final AbsoluteEncoder m_azimuthEnc;
    private final SparkPIDController m_azimuthPID;
    private Slot0Configs slot0Configs = new Slot0Configs();
    private final VelocityVoltage m_request = new VelocityVoltage(0.0).withSlot(0);

    /**
     * Create a new FRC 1706 NEOKrakenSwerveModule Object
     *
     * @param moduleID  module ID also CAN ID for Azimuth and Drive MCs.
     * @param offset  The offset for the analog encoder.
     */
    public NEOKrakenSwerveModule(int moduleID, double offset) {
        configurePID();
        m_driveMotor = new TalonFX(moduleID,"*");
        m_driveMotor.getConfigurator().apply(slot0Configs);
        m_driveMotor.getConfigurator()
                .apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(CurrentLimit.kDriveSupply)
                        .withStatorCurrentLimit(CurrentLimit.kDriveStator)
                        .withSupplyCurrentLimitEnable(true)
                        .withStatorCurrentLimitEnable(true));
        m_driveMotor.setNeutralMode(NeutralModeValue.Brake);

        m_driveMotor.getConfigurator().apply(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.100));

        m_azimuthMotor = new CANSparkMax(moduleID, MotorType.kBrushless);
        //m_azimuthMotor.restoreFactoryDefaults();
        m_azimuthMotor.setSmartCurrentLimit(CurrentLimit.kAzimuth);
        m_azimuthMotor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_azimuthMotor.setInverted(true);
        m_azimuthMotor.setIdleMode(IdleMode.kBrake);

        m_azimuthEnc = m_azimuthMotor.getAbsoluteEncoder(Type.kDutyCycle);
        m_azimuthEnc.setPositionConversionFactor(Aziumth.kPositionFactor);
        m_azimuthEnc.setVelocityConversionFactor(Aziumth.kVelocityFactor);

        //m_azimuthEnc.setZeroOffset(offset);

        m_azimuthEnc.setInverted(true);



        m_azimuthPID = m_azimuthMotor.getPIDController();

        m_azimuthPID.setFeedbackDevice(m_azimuthEnc);

        m_azimuthPID.setPositionPIDWrappingEnabled(true);
        m_azimuthPID.setPositionPIDWrappingMinInput(0.0);
        m_azimuthPID.setPositionPIDWrappingMaxInput(2.0 * Math.PI);

        m_azimuthPID.setP(Aziumth.kp);

        m_azimuthMotor.burnFlash();
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
        slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
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
        m_azimuthPID.setReference(state.angle.getRadians(), ControlType.kPosition);
    }

    public double getStateAngle() {
        return m_azimuthEnc.getPosition();
    }

    public void stop() {
        m_driveMotor.stopMotor();
        m_azimuthMotor.stopMotor();
    }

}
