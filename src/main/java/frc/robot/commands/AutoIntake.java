package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class AutoIntake extends Command {
    private final Intake m_intake;
    private final Indexer m_indexer;
    private final Feeder m_feeder;
    private final Timer m_timer = new Timer();
    private boolean m_finishedIntake = false;
    private boolean m_currentSpiked = false;
    private final boolean m_direction;

    public AutoIntake(Intake intake, Indexer indexer, Feeder feeder, boolean direction){
        m_intake = intake;
        m_indexer = indexer;
        m_feeder = feeder;
        m_direction = direction;

    }
    

@Override
public void initialize(){
    SmartDashboard.putNumber("Feeder Encoder", m_feeder.getEncoder());
    m_timer.reset();
    m_timer.start();
    m_currentSpiked = false;
    m_finishedIntake = false;
    m_indexer.run(0.8);
    m_feeder.run(0.4);
    if(m_direction){
        m_intake.run(1.0, -1.0);
    }
    else{
        m_intake.run(1.0, 1.0);
    }
}

@Override
public void execute(){
    if(m_feeder.getCurrent() > 12.0 && !m_currentSpiked && m_timer.get() > 0.5){
        m_currentSpiked = true;
        m_indexer.run(.2);
        m_intake.run(.2);
        m_feeder.setZero();
        m_feeder.setPose(2.0);
    }
    if (m_currentSpiked && m_feeder.atSetpoint()){
        m_finishedIntake = true;
    }


   
}

@Override
public boolean isFinished() {
    return m_finishedIntake;
}
@Override
public void end(boolean interrupted){
    m_intake.stop();
    m_indexer.stop();
    m_feeder.stop();
}

}