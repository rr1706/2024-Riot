package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;

public class Elevator extends SubsystemBase{
    private final SparkMax m_leftMotor = new SparkMax(14, MotorType.kBrushless);
    private final SparkMax m_rightMotor = new SparkMax(15, MotorType.kBrushless);

    private final RelativeEncoder m_leftEnc = m_leftMotor.getEncoder();
    private final RelativeEncoder m_rightEnc = m_rightMotor.getEncoder();

    private final ProfiledPIDController m_pid = new ProfiledPIDController(0.05,0.01,0.00, 
                                                    new Constraints(75, 125));

    private final ElevatorFeedforward m_leftFF = new ElevatorFeedforward(0.025, 0.02, 1.0/183.0);
    private final ElevatorFeedforward m_rightFF = new ElevatorFeedforward(0.025, 0.02, 1.0/183.0);

    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State(3.0,0.0);

    private boolean m_PIDEnabled = false;

    public Elevator() {
        motorConfigs();

        m_setpoint = new TrapezoidProfile.State(getLeftPose(),0.0);
    }

    public void zero() {
        m_PIDEnabled = false;

        m_leftMotor.set(-0.25);
        m_rightMotor.set(-0.25);
    }

    public void set(double power) {
        m_leftMotor.set(power/60);
        m_rightMotor.set(power/60);
    }

    public void setPose(double pose) {
        m_PIDEnabled = true;

        m_pid.reset(new TrapezoidProfile.State(m_rightEnc.getPosition(), m_rightEnc.getVelocity()));
        m_setpoint = new TrapezoidProfile.State(pose, 0.0);
    }

    public Command setPoseCmd(double pose) {
        return runOnce(()-> setPose(pose));
    }

    public void setZero() {
        m_rightEnc.setPosition(0.0);
        m_leftEnc.setPosition(0.0);
    }

    public double getLeftCurrent() {
        return m_leftMotor.getOutputCurrent();
    }

    public double getRightCurrent() {
        return m_rightMotor.getOutputCurrent();
    }

    public double getLeftPose() {
        return m_leftEnc.getPosition();
    }

    public double getRightPose() {
        return m_rightEnc.getPosition();
    }

    public void stop(){
        m_leftMotor.stopMotor();
        m_rightMotor.stopMotor();
    }

    @Override
    public void periodic() {
        double output = m_pid.calculate(getLeftPose(), m_setpoint);
        TrapezoidProfile.State state = m_pid.getSetpoint();

        if(m_PIDEnabled) {
            m_leftMotor.set(output + m_leftFF.calculate(state.velocity));
            m_rightMotor.set(output + m_rightFF.calculate(state.velocity));
        }

        SmartDashboard.putNumber("Elevator Setpoint", state.position);
        SmartDashboard.putNumber("Elevator Right Position", getRightPose());
        SmartDashboard.putNumber("Elevator Left Position", getLeftPose());
    }

    public void motorConfigs() {
        SparkMaxConfig leftConfigs = new SparkMaxConfig();
        SparkMaxConfig rightConfigs = new SparkMaxConfig();

        leftConfigs.smartCurrentLimit(CurrentLimit.kElevator);
        leftConfigs.voltageCompensation(GlobalConstants.kVoltCompensation);
        
        leftConfigs.idleMode(IdleMode.kBrake);
        leftConfigs.inverted(true);

        m_leftMotor.configure(leftConfigs, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        rightConfigs.smartCurrentLimit(CurrentLimit.kElevator);
        rightConfigs.voltageCompensation(GlobalConstants.kVoltCompensation);
        
        rightConfigs.idleMode(IdleMode.kBrake);
        rightConfigs.inverted(false);
        
        m_rightMotor.configure(rightConfigs, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}