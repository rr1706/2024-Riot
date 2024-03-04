package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;

public class ReverseFeed extends Command {
    private final Feeder m_feeder;
    private final Indexer m_indexer;

    public ReverseFeed(Feeder feeder, Indexer indexer){
        m_feeder = feeder;
        m_indexer = indexer;
        addRequirements(feeder, indexer);
    }

    @Override
    public void initialize() {
        m_feeder.run(-0.4);
        m_indexer.run(-0.4);
    }
    @Override
    public void end(boolean interrupted) {
        m_feeder.stop();
        m_indexer.stop();
    }
}
