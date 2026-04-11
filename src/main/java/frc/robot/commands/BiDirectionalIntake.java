package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class BiDirectionalIntake extends Command {
    private final Intake m_intake;
    private final Drivetrain m_robotDrive;
    private final Indexer m_indexer;
    private final Feeder m_feeder;
    private final CommandXboxController m_controller;
    private final Timer m_timer = new Timer();
    private boolean noteInRobot = false;
    private double noteTime = Double.POSITIVE_INFINITY;

    public BiDirectionalIntake(Intake intake, Drivetrain robotDrive, Indexer indexer, Feeder feeder,
            CommandXboxController controller) {
        m_intake = intake;
        m_robotDrive = robotDrive;
        m_indexer = indexer;
        m_feeder = feeder;
        m_controller = controller;

        addRequirements(feeder, indexer);

    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        noteInRobot = false;
        noteTime = Double.POSITIVE_INFINITY;
        m_indexer.run(0.7);
        m_feeder.run(0.8);
        double robotVelocity = m_robotDrive.getChassisSpeed().vxMetersPerSecond;
        m_intake.run(1.0, robotVelocity);
    }

    @Override
    public void execute() {
        double time = m_timer.get();
        double robotVelocity = m_robotDrive.getChassisSpeed().vxMetersPerSecond;
       if (!m_feeder.getProx() && noteInRobot) {
            noteInRobot = false;
            noteTime = Double.POSITIVE_INFINITY;
            m_intake.run(1.0, robotVelocity);
            m_indexer.run(0.7);
            m_feeder.run(0.8);
        } else if (m_feeder.getProx() && !noteInRobot) {
            noteInRobot = true;
            noteTime = time;
            m_controller.getHID().setRumble(RumbleType.kBothRumble, 1.0);
            m_intake.run(0.2, robotVelocity);
            m_feeder.run(0.6);
        } else if (time >= (noteTime + 0.110)) {
            m_feeder.run(0.4);
            m_indexer.run(-1.0);
            m_intake.run(0.6, robotVelocity);
        }else{
             m_intake.run(1.0, robotVelocity);
        }

    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        m_intake.stop();
        m_indexer.stop();
        m_feeder.stop();
        m_controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);

    }

}