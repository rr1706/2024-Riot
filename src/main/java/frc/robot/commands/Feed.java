package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class Feed extends Command {
    private final Feeder m_feed;
    private final Indexer m_indexer;
    private final Intake m_intake;
    private final Timer m_timer = new Timer();

    public Feed(Feeder feed, Indexer indexer, Intake intake){
        m_feed = feed;
        m_intake = intake;
        m_indexer = indexer;
        addRequirements(m_feed);
    }

    @Override
    public void initialize() {
        m_feed.run(0.6);
        m_indexer.run(0.6);
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        m_feed.stop();
        m_indexer.stop();
    }
}
