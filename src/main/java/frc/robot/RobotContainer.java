// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PitcherConstants;
import frc.robot.commands.BiDirectionalIntake;
import frc.robot.commands.DriveByController;
import frc.robot.commands.Feed;
import frc.robot.commands.Handoff;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.ZeroElevator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pitcher;
import frc.robot.subsystems.Shooter;


public class RobotContainer {
    private final Drivetrain m_drive = new Drivetrain();
    private final Shooter m_shooter = new Shooter();
    private final Feeder m_feeder = new Feeder();
    private final Intake m_intaker = new Intake();
    private final Indexer m_indexer = new Indexer();
    private final Pitcher m_pitcher = new Pitcher();
    private final Climber m_climber = new Climber();
    private final Elevator m_elevator = new Elevator();
    private final Manipulator m_manipulator = new Manipulator();

    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);

    private final DriveByController m_driveByController = new DriveByController(m_drive, m_driverController);

    public RobotContainer() {
        m_driveByController.changeFieldOrient();
        m_drive.setDefaultCommand(m_driveByController);
        configureBindings();

    }

    private void configureBindings() {
        m_driverController.leftBumper()
                .onTrue(new InstantCommand(() -> m_pitcher.pitchToAngle(12.0))
                        .alongWith(new InstantCommand(() -> m_shooter.run(40.0, 25.0))))
                .onFalse(new InstantCommand(() -> m_pitcher.pitchToAngle(PitcherConstants.kHome))
                        .alongWith(new InstantCommand(() -> m_shooter.stop())));

        m_driverController.rightBumper()
                .onTrue(new InstantCommand(() ->{
                        m_shooter.run(-10.0, 0.0);
                        m_feeder.run(-0.2);
                })).onFalse(new InstantCommand(()->{
                        m_shooter.stop();
                        m_feeder.stop();
                }));

        m_driverController.rightTrigger(0.25)
                .onTrue(new InstantCommand(() -> m_feeder.run(0.6))
                        .alongWith(new InstantCommand(() -> m_indexer.run(0.6))))
                .onFalse(new InstantCommand(() -> m_feeder.stop())
                        .alongWith(new InstantCommand(() -> m_indexer.stop())));

        m_driverController.y().onTrue(m_pitcher.changePitch(2.0));
        m_driverController.b().onTrue(m_pitcher.changePitch(-2.0));

        m_driverController.pov(0).onTrue(m_shooter.changeSpeed(10.0));
        m_driverController.pov(180).onTrue(m_shooter.changeSpeed(-10.0));

        m_driverController.start().onTrue(new InstantCommand(
                () -> m_drive.resetOdometry(new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(Math.PI)))));

        m_driverController.rightTrigger(0.25).whileTrue(new Feed(m_feeder, m_indexer));

        m_driverController.leftBumper()
                .whileTrue(new BiDirectionalIntake(m_intaker, m_drive, m_indexer, m_feeder, m_driverController))
                .onFalse(new ReverseIntake(m_feeder, m_indexer, m_intaker));

        m_driverController.rightBumper()
                .whileTrue(
                        new Handoff(m_intaker, m_indexer, m_manipulator, m_feeder, -14.0, m_elevator, m_drive, false));

        m_driverController.back()
                .onTrue(new InstantCommand(() -> m_climber.setPose(70.0)).alongWith(new ZeroElevator(m_elevator)));

        m_driverController.pov(90).onTrue(new InstantCommand(() -> m_manipulator.run(-0.2)))
                .onFalse(new InstantCommand(() -> m_manipulator.stop()));

        m_driverController.pov(270)
                .onTrue(new Handoff(m_intaker, m_indexer, m_manipulator, m_feeder, -19.8, m_elevator, m_drive, false)
                        .withTimeout(2.0));

        m_driverController.a().onTrue(new InstantCommand(() -> m_elevator.setPose(28.0)))
                .onFalse(new InstantCommand(() -> m_elevator.setPose(3.0)).andThen(new WaitCommand(0.25))
                        .andThen(new ZeroElevator(m_elevator)));
    }
}
