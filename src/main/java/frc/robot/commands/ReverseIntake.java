package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class ReverseIntake extends Command {
    private final Feeder m_feeder;
    private final Indexer m_indexer;
    private final Intake m_intake;

    public ReverseIntake(Feeder feeder, Indexer indexer, Intake intake) {
        m_feeder = feeder;
        m_indexer = indexer;
        m_intake = intake;
        addRequirements(feeder, indexer);
    }

    @Override
    public void initialize() {
        m_feeder.run(-0.4);
        m_indexer.run(-0.4);
        m_intake.run(-0.2);
    }

    @Override
    public void end(boolean interrupted) {
        m_feeder.stop();
        m_indexer.stop();
        m_intake.stop();
    }
}
