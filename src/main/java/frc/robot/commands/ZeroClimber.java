package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ZeroClimber extends Command {
    private final Climber m_climber;
    private final Timer m_timer = new Timer();
    private boolean m_leftFinished = false;
    private boolean m_rightFinished = false;

    public ZeroClimber(Climber climber) {
        m_climber = climber;
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        m_climber.zero();
        m_leftFinished = false;
        m_rightFinished = false;

    }

    @Override
    public void execute() {
        if (m_timer.get() > 0.2) {
            if (m_climber.getLeftCurrent() > 30.0) {
                m_climber.stopLeft();
                m_climber.setLeftZero();
                m_leftFinished = true;
            }
            if (m_climber.getRightCurrent() > 30.0) {
                m_climber.stopRight();
                m_climber.setRightZero();
                m_rightFinished = true;
            }

        }
    }

    @Override
    public boolean isFinished() {
        return m_leftFinished && m_rightFinished;
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.setPose(0.0);
        m_timer.stop();
    }

}
