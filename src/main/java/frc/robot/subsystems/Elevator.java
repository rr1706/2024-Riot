
package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;

public class Elevator extends SubsystemBase{
    private final CANSparkMax m_motorL = new CANSparkMax(14, MotorType.kBrushless);
    private final CANSparkMax m_motorR = new CANSparkMax(15, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motorL.getEncoder();
    private final RelativeEncoder m_encoderR = m_motorR.getEncoder();

    private final ProfiledPIDController m_pid = new ProfiledPIDController(0.05,0.01
    ,0.00, new Constraints(50, 100));

    private final ElevatorFeedforward m_ff = new ElevatorFeedforward(0.025, 0.02, 1.0/100.0);
    private final ElevatorFeedforward m_ff2 = new ElevatorFeedforward(0.025, 0.02, 1.0/100.0);

    private boolean m_PIDEnabled = false;
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State(2.5,0.0);


    public Elevator() {
        m_motorL.setSmartCurrentLimit(CurrentLimit.kElevator);
        m_motorR.setSmartCurrentLimit(CurrentLimit.kElevator);
        m_motorL.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_motorR.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_motorL.setIdleMode(IdleMode.kBrake);
        m_motorR.setIdleMode(IdleMode.kBrake);

        m_motorL.setInverted(true);
        m_motorR.setInverted(false);

        m_encoder.setVelocityConversionFactor(1.0/60.0);

        m_motorL.burnFlash();
        m_motorR.burnFlash();

        m_setpoint= new TrapezoidProfile.State(getLeftPose(),0.0);
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


    }

    public void zero(){
        m_PIDEnabled = false;
        m_motorL.set(-0.25);
        m_motorR.set(-0.25);
    }

    public void set(double power){
        m_motorL.set(power);
        m_motorR.set(power);
    }

    public void setPose(double pose){
        m_PIDEnabled = true;
        m_pid.reset(new TrapezoidProfile.State(m_encoder.getPosition(),m_encoder.getVelocity()));
        m_setpoint = new TrapezoidProfile.State(pose,0.0);
    }

    public void setZero(){
        m_encoder.setPosition(0.0);
        m_encoderR.setPosition(0.0);
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
