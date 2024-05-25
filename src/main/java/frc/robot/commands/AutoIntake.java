package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoIntake extends Command {
    private final Intake m_intake;
    private final Indexer m_indexer;
    private final Feeder m_feeder;
    private final Shooter m_shooter;
    private final Timer m_timer = new Timer();
    private boolean m_finishedIntake = false;
    private boolean m_currentSpiked = false;
    private final boolean m_direction;

    public AutoIntake(Intake intake, Indexer indexer, Feeder feeder, boolean direction, Shooter shooter){
        m_intake = intake;
        m_indexer = indexer;
        m_feeder = feeder;
        m_shooter = shooter;
        m_direction = direction;
        

    }
    

@Override
public void initialize(){
    SmartDashboard.putNumber("Feeder Encoder", m_feeder.getEncoder());
    m_timer.reset();
    m_timer.start();
    m_currentSpiked = false;
    m_finishedIntake = false;
    m_indexer.run(0.6);
    m_feeder.run(0.6);
    m_shooter.stop();
    if(m_direction){
        m_intake.run(1.0, -1.0);
    }
    else{
        m_intake.run(1.0, 1.0);
    }
}

@Override
public void execute(){
    if(m_intake.getKickerCurrent() > 15.0 && !m_currentSpiked && m_timer.get() > 0.4){
        m_currentSpiked = true;
        m_timer.reset();
        m_timer.start();
    }
    else if(m_currentSpiked && m_timer.get() > 0.200){
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
    m_shooter.stop();

}

}