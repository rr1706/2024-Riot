package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PitcherConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
    private final Drivetrain m_drive = new Drivetrain();
    private final Shooter m_shooter = new Shooter();
    private final Feeder m_feeder = new Feeder();
    private final Intake m_intaker = new Intake();
    private final Indexer m_indexer = new Indexer();
    private final Pitcher m_pitcher = new Pitcher();
    private final Elevator m_elevator = new Elevator();
    private final Manipulator m_manipulator = new Manipulator();

    private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    private final DriveByController m_driveByController = new DriveByController(m_drive, m_driverController);
    private final BiDirectionalIntake m_intakeCommand = new BiDirectionalIntake(m_intaker, m_drive, m_indexer, m_feeder, m_driverController);
    private final ReverseIntake m_reverseIntakeCommand = new ReverseIntake(m_feeder, m_indexer, m_intaker);
    private final Handoff m_handoffCommand = new Handoff(m_intaker, m_indexer, m_manipulator, m_feeder, -19.8, m_elevator, m_drive, false);

    public RobotContainer() {
        m_drive.setDefaultCommand(m_driveByController);
        configureBindings();
    }

    private void configureBindings() {
        m_driverController.start().onTrue((new InstantCommand(()-> m_drive.resetOdometry(new Pose2d()))));

        m_driverController.leftTrigger(0.25)
                .onTrue(m_pitcher.pitchToAngleCmd(2).alongWith(m_shooter.runShooter(10, 25.0)))
                .onFalse(m_pitcher.pitchToAngleCmd(PitcherConstants.kHome).alongWith(m_shooter.stopCmd()));

        m_driverController.rightTrigger(0.25)
                .whileTrue(m_feeder.runCmd(0.6).alongWith(m_indexer.runCmd(0.6)));

        m_driverController.y().onTrue(m_pitcher.changePitch(2.0));
        m_driverController.b().onTrue(m_pitcher.changePitch(-2.0));

        m_driverController.povUp().onTrue(m_shooter.changeSpeed(10.0));
        m_driverController.povDown().onTrue(m_shooter.changeSpeed(-10.0));

        m_driverController.leftBumper().whileTrue(m_intakeCommand)
                .onFalse((m_reverseIntakeCommand.alongWith(new ConditionalCommand(new WaitCommand(1.0), 
                        m_shooter.runShooter(-10.0, 0), m_driverController.rightTrigger()::getAsBoolean)))
                                .withTimeout(0.130));

        m_driverController.povLeft().whileTrue(m_manipulator.runCmd(-0.2));

        m_driverController.povRight().onTrue(m_handoffCommand.withTimeout(2.0));

        m_driverController.a().onTrue(m_elevator.setPoseCmd(28.0))
                .onFalse(m_elevator.setPoseCmd(3.0).andThen(new WaitCommand(0.25)).andThen(new ZeroElevator(m_elevator)));
    }

    public Command getAutonomousCommand() {
        return new WaitCommand(0.0);
    }
}