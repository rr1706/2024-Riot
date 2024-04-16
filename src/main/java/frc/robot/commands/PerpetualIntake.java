package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class PerpetualIntake extends Command {
    private final Intake m_intake;
    private final Indexer m_indexer;
    private final Feeder m_feeder;
    private double m_direction;

    public PerpetualIntake(Intake intake, Indexer indexer, Feeder feeder, double direction){
        m_intake = intake;
        m_indexer = indexer;
        m_feeder = feeder;
        m_direction = direction;
    }
    

@Override
public void initialize(){
    m_indexer.run(0.7);
    m_feeder.run(0.7);
    m_intake.run(1.0, m_direction);

}

@Override
public void end(boolean interrupted){
    m_intake.stop();
    m_indexer.stop();
    m_feeder.stop();
}

}