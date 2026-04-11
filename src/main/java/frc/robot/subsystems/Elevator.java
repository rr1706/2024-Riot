
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;

public class Elevator extends SubsystemBase{
    private final SparkMax m_motorL = new SparkMax(14, MotorType.kBrushless);
    private final SparkMax m_motorR = new SparkMax(15, MotorType.kBrushless);

    private final RelativeEncoder m_encoderL = m_motorL.getEncoder();
    private final RelativeEncoder m_encoder = m_motorR.getEncoder();

    private final ProfiledPIDController m_pid = new ProfiledPIDController(0.05,0.01
    ,0.00, new Constraints(75, 125));

    private final ElevatorFeedforward m_ff = new ElevatorFeedforward(0.025, 0.02, 1.0/183.0);
    private final ElevatorFeedforward m_ff2 = new ElevatorFeedforward(0.025, 0.02, 1.0/183.0);

    private boolean m_PIDEnabled = false;

    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State(3.0,0.0);

    private final SparkMaxConfig m_motorLConfig = new SparkMaxConfig();
    private final SparkMaxConfig m_motorRConfig = new SparkMaxConfig();

    public Elevator() {
        m_motorLConfig.smartCurrentLimit(CurrentLimit.kElevator);
        m_motorRConfig.smartCurrentLimit(CurrentLimit.kElevator);

        m_motorLConfig.voltageCompensation(GlobalConstants.kVoltCompensation);
        m_motorRConfig.voltageCompensation(GlobalConstants.kVoltCompensation);
        
        m_motorLConfig.idleMode(IdleMode.kBrake);
        m_motorRConfig.idleMode(IdleMode.kBrake);

        m_motorLConfig.inverted(true);
        m_motorRConfig.inverted(false);

        m_motorL.configure(m_motorLConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_motorR.configure(m_motorRConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        m_setpoint = new TrapezoidProfile.State(getLeftPose(),0.0);
    }
    
    @Override
    public void periodic() {
        double output = m_pid.calculate(getLeftPose(), m_setpoint);

        TrapezoidProfile.State state = m_pid.getSetpoint();

        double ff = m_ff.calculate(state.velocity);
        double ff2 = m_ff2.calculate(state.velocity);
        if(m_PIDEnabled){
            m_motorL.set(output+ff);
            m_motorR.set(output+ff2);
        }

        SmartDashboard.putNumber("Ele Desired Pose", state.position);
        SmartDashboard.putNumber("Ele Actual Pose", m_encoder.getPosition());
        SmartDashboard.putNumber("Ele Pose 2", m_encoderL.getPosition());
    }

    public void zero(){
        m_PIDEnabled = false;
        m_motorL.set(-0.25);
        m_motorR.set(-0.25);
    }

    public void set(double power){
        m_motorL.set(power/60);
        m_motorR.set(power/60);
    }

    public void setPose(double pose){
        m_PIDEnabled = true;
        m_pid.reset(new TrapezoidProfile.State(m_encoder.getPosition(),m_encoder.getVelocity()));
        m_setpoint = new TrapezoidProfile.State(pose,0.0);
    }

    public void setZero(){
        m_encoder.setPosition(0.0);
        m_encoderL.setPosition(0.0);
    }

    public double 
    getLeftCurrent(){
        return m_motorL.getOutputCurrent();
    }

    public double getRightCurrent(){
        return m_motorR.getOutputCurrent();
    }

    public double getLeftPose(){
        return m_encoder.getPosition();
    }

    public void stopLeft(){
        m_motorL.stopMotor();
    }
    public void stop(){
        m_motorL.stopMotor();
    }
}
