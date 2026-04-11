package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class MoveToFloor extends Command {
    private final Feeder m_feeder;
    private final Indexer m_indexer;
    private final Intake m_intake;
    private final boolean m_back;

    public MoveToFloor(Feeder feeder, Indexer indexer, Intake intake, boolean back) {
        m_feeder = feeder;
        m_indexer = indexer;
        m_intake = intake;
        m_back = back;
        addRequirements(feeder, indexer);
    }

    @Override
    public void initialize() {
        m_feeder.run(-0.6);
        m_indexer.run(-0.6);
        if(m_back){
            m_intake.runIndividual(-0.6*0.7,-0.6);
        }else{
            m_intake.runIndividual(0.6*0.7,-0.6);    
        }
    }

    @Override
    public void end(boolean interrupted) {
        // m_feeder.stop();
        // m_indexer.stop();
        // m_intake.stop();
    }
}

