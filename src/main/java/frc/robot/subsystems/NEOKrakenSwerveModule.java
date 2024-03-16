package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final RelativeEncoder m_azimuthEnc;
    private final AnalogEncoder m_absEncoder;
    private final double m_offset;
    private final SparkPIDController m_azimuthPID;
    private double moduleID;
    private double m_referenceAngleRadians = 0;
    private Slot0Configs slot0Configs = new Slot0Configs();
    private final VelocityVoltage m_request = new VelocityVoltage(0.0).withSlot(0);
    private boolean m_useNEOEncoder = false;

      private final PIDController m_azimuthRioPID = 
        new PIDController(Aziumth.rioKp,Aziumth.rioKi,Aziumth.rioKd);

  /**
   * Create a new FRC 1706 NEOKrakenSwerveModule Object
   *
   * @param drive The drive SparkMax CAN ID.
   * @param azimuth The azimuth SparkMax CAN ID.
   * @param absEnc The analog encoder port for the absolute encoder. 
   * @param offset The offset for the analog encoder. 
   */
    public NEOKrakenSwerveModule(int moduleID, double offset) {
        this.moduleID = moduleID;
        configurePID();
        m_driveMotor = new TalonFX(moduleID);
        m_driveMotor.getConfigurator().apply(slot0Configs);
        m_driveMotor.getConfigurator()
            .apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(CurrentLimit.kDriveSupply)
            .withStatorCurrentLimit(CurrentLimit.kDriveStator)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimitEnable(true));
        m_driveMotor.setNeutralMode(NeutralModeValue.Brake);

        m_azimuthMotor = new CANSparkMax(moduleID, MotorType.kBrushless);
        m_azimuthMotor.setSmartCurrentLimit(CurrentLimit.kAzimuth);
        m_azimuthMotor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_azimuthMotor.setInverted(true);
        m_azimuthMotor.setIdleMode(IdleMode.kBrake);

        m_azimuthEnc = m_azimuthMotor.getEncoder();
        m_azimuthEnc.setPositionConversionFactor(Aziumth.kPositionFactor);
        m_azimuthEnc.setVelocityConversionFactor(Aziumth.kVelocityFactor);
        m_azimuthEnc.setAverageDepth(4);
        m_azimuthEnc.setMeasurementPeriod(16);

        m_azimuthPID = m_azimuthMotor.getPIDController();
        m_azimuthPID.setP(Aziumth.kp);

        m_azimuthRioPID.enableContinuousInput(-Math.PI, Math.PI);

        m_absEncoder = new AnalogEncoder(moduleID-1);
        m_offset = offset;

        m_azimuthEnc.setPosition(getAbsEncoder());

        m_azimuthMotor.burnFlash();
        SmartDashboard.putBoolean("Use NEO Encoder", m_useNEOEncoder);
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

    public void configurePID(){
        slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.0001; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative
    }

    public double getDriveVelocity(){
       return m_driveMotor.getVelocity().getValueAsDouble()*Drive.kToMeters;
    }

    public double getDrivePosition(){
       return m_driveMotor.getPosition().getValueAsDouble()*Drive.kToMeters;
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getStateAngle()));
        // Calculate the drive output from the drive PID controller.
        // final double driveOutput =
        // m_drivePIDController.calculate(m_driveEncoder.getVelocity(),
        // state.speedMetersPerSecond);
        // Calculates the desired feedForward motor % from the current desired velocity
        // and the static and feedforward gains
        double metersToRotations = state.speedMetersPerSecond * Drive.kToRots;
        SmartDashboard.putNumber("Module"+moduleID, state.speedMetersPerSecond);
        SmartDashboard.putNumber("Kraken"+moduleID, getDriveVelocity());
        m_driveMotor.setControl(m_request.withVelocity(metersToRotations).withSlot(0));

        m_useNEOEncoder = SmartDashboard.getBoolean("Use NEO Encoder", false);

        if(m_useNEOEncoder){
            setReferenceAngle(state.angle.getRadians());
        }
        else{
            m_azimuthMotor.set(m_azimuthRioPID.calculate(getAbsEncoder(),state.angle.getRadians()));
        }

        

    }

    /**
     * Sets the reference angle for the azimuth.
     *
     * @param referenceAngleRadians Desired reference angle.
     */
    public void setReferenceAngle(double referenceAngleRadians) {
        double currentAngleRadians = m_azimuthEnc.getPosition();

        double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI;
        }

        // The reference angle has the range [0, 2pi) but the Neo's encoder can go above
        // that
        double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
        if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }

        m_referenceAngleRadians = referenceAngleRadians;
        m_azimuthPID.setReference(adjustedReferenceAngleRadians, ControlType.kPosition);
    }

    /**
     * Gets the reference angle for the azimuth.
     *
     * @return reference angle.
     */
    public double getReferenceAngle() {
        return m_referenceAngleRadians;
    }

    public double getStateAngle() {
        double motorAngleRadians = m_azimuthEnc.getPosition();
        motorAngleRadians %= 2.0 * Math.PI;
        if (motorAngleRadians < 0.0) {
            motorAngleRadians += 2.0 * Math.PI;
        }
        return getAbsEncoder();
        //return motorAngleRadians;
    }

    public double getAbsEncoder() {
        double absEnc = m_absEncoder.getAbsolutePosition() * 2 * Math.PI + m_offset;
        return absEnc;
    }


    public void stop() {
        m_driveMotor.stopMotor();
        m_azimuthMotor.stopMotor();
    }

}
