package frc.robot.commands;

import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class BiDirectionalIntake extends Command {
    private final Intake m_intake;
    private final Drivetrain m_robotDrive;
    private final Indexer m_indexer;
    private final Feeder m_feeder;
    private double robotVelocity = 1;

    public BiDirectionalIntake(Intake intake, Drivetrain robotDrive, Indexer indexer, Feeder feeder){
        m_intake = intake;
        m_robotDrive = robotDrive;
        m_indexer = indexer;
        m_feeder = feeder;

    }
    

@Override
public void initialize(){
    SmartDashboard.putNumber("Velocity", m_robotDrive.getChassisSpeed().vxMetersPerSecond);
}

@Override
public void execute(){
    robotVelocity = m_robotDrive.getChassisSpeed().vxMetersPerSecond;
        m_intake.run(1.0, robotVelocity);
        m_indexer.run(0.8);
        m_feeder.run(0.4);
}

@Override
public void end(boolean interrupted){
    m_intake.stop();
    m_indexer.stop();
    m_feeder.stop();
}

}