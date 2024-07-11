package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class PerpetualIntake extends Command {
    private final Intake m_intake;
    private final Indexer m_indexer;
    private final Feeder m_feeder;
    private boolean intakedNote = false;
    private double m_direction;
    private final Timer m_timer = new Timer();

    public PerpetualIntake(Intake intake, Indexer indexer, Feeder feeder, double direction) {
        m_intake = intake;
        m_indexer = indexer;
        m_feeder = feeder;
        m_direction = direction;
        SmartDashboard.putBoolean("Intaked Note", intakedNote);
    }

    @Override
    public void initialize() {
        m_indexer.run(0.6);
        m_feeder.run(0.6);
        m_intake.run(1.0, m_direction);
        intakedNote = false;
        m_timer.reset();
        m_timer.start();

    }

    @Override
    public void execute() {
        if (!intakedNote && m_intake.getKickerCurrent() >= 10.0 && m_timer.get() > 0.2) {
            intakedNote = true;
        }
    }

    public boolean didIntakeNote() {
        return intakedNote;
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
        m_indexer.stop();
        m_feeder.stop();
        SmartDashboard.putBoolean("Intaked Note", intakedNote);
    }

}