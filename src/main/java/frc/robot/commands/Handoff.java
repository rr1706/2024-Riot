package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;

public class Handoff extends Command{
    private final Intake m_intake;
    private final Indexer m_indexer;
    private final Manipulator m_manipulator;
    private final Feeder m_feeder;
    private final Timer m_timer = new Timer();
    private boolean m_finished = false;
    private boolean currentSpiked = false;
    private double m_setpoint;

    public Handoff(Intake intake, Indexer indexer, Manipulator manipulator, Feeder feeder, double setpoint){
        m_intake = intake;
        m_indexer = indexer;
        m_manipulator = manipulator;
        m_feeder = feeder;
        m_setpoint = setpoint;
    }

    @Override
    public void initialize() {
        m_finished = false;
        currentSpiked = false;
        m_timer.reset();
        m_timer.start();
        m_feeder.run(-0.5);
        m_intake.run(.5);
        m_indexer.run(-.5);
        m_manipulator.run(-.3);
    }

    @Override
    public void execute() {
        if(m_timer.get() > 0.2 && m_manipulator.getCurrent() > 15.0 && !currentSpiked){
            currentSpiked = true;
            m_manipulator.setZero();
            m_manipulator.setPose(m_setpoint);
        }

        if(currentSpiked && m_manipulator.atSetpoint()){
            m_finished = true;
        }
        
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }

    @Override
    public void end(boolean interrupted) {
        m_feeder.stop();
        m_intake.stop();
        m_indexer.stop();
        m_manipulator.stop();

    }
    
}
