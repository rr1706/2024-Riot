package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ZeroElevator extends Command {
    private final Elevator m_elevator;
    private final Timer m_timer = new Timer();
    private boolean m_finished = false;

    public ZeroElevator(Elevator elevator) {
        m_elevator = elevator;
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        m_elevator.zero();
        m_finished = false;

    }

    @Override
    public void execute() {
        if (m_timer.get() > 0.1) {
            if (m_elevator.getLeftCurrent() + m_elevator.getRightCurrent() > 40.0) {
                m_elevator.stop();
                m_elevator.setZero();
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
        m_elevator.setPose(2.5);
        m_timer.stop();
    }

}
