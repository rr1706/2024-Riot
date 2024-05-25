package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pitcher;

public class ZeroPitcher extends Command {
    private final Pitcher m_pitcher;
    private final Timer m_timer = new Timer();
    private boolean m_finished = false;

    public ZeroPitcher(Pitcher pitcher) {
        m_pitcher = pitcher;
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        m_pitcher.zero();
        m_finished = false;

    }

    @Override
    public void execute() {
        if (m_timer.get() > 0.2) {
            if (m_pitcher.getCurrent() > 20.0) {
                m_pitcher.stop();
                m_pitcher.setZero();
                m_finished = true;

            }
        }

    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }

    @Override
    public void end(boolean interrupted) {
        m_pitcher.pitchToAngle(2.0);
        m_timer.stop();
    }

}
