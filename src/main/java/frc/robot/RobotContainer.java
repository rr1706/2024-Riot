// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants.Auto;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PitcherConstants;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoIntakeAimAssist;
import frc.robot.commands.AutoShooterByPose;
import frc.robot.commands.AutoMoveNShoot;
import frc.robot.commands.BiDirectionalIntake;
import frc.robot.commands.CheckForNote;
import frc.robot.commands.DriveByController;
import frc.robot.commands.Feed;
import frc.robot.commands.Handoff;
import frc.robot.commands.MoveToFloor;
import frc.robot.commands.PerpetualIntake;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.RotateToAngle;
import frc.robot.commands.SmartShootByPose;
import frc.robot.commands.ZeroClimber;
import frc.robot.commands.ZeroElevator;
import frc.robot.commands.ZeroPitcher;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pitcher;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    private final SendableChooser<Command> autoChooser;

    private final Drivetrain m_drive = new Drivetrain();
    private final Shooter m_shooter = new Shooter();
    private final Feeder m_feeder = new Feeder();
    private final Intake m_intaker = new Intake();
    private final Indexer m_indexer = new Indexer();
    private final Pitcher m_pitcher = new Pitcher();
    private final Climber m_climber = new Climber();
    private final Elevator m_elevator = new Elevator();
    private final Manipulator m_manipulator = new Manipulator();
    private final LEDs m_led = new LEDs(m_feeder::getProx);

    private final PoseEstimator m_poseEstimator = new PoseEstimator(m_drive);

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);

    private final CommandXboxController m_demoController = new CommandXboxController(
            OperatorConstants.kDemoControllerPort);

    private final CommandGenericHID m_operatorPanel = new CommandGenericHID(1);

    private final DriveByController m_driveByController = new DriveByController(m_drive, m_driverController);

    private final Command m_teleInitCommand = new InstantCommand(() -> {
        m_shooter.stop();
        m_pitcher.pitchToAngle(2.0);
        m_poseEstimator.setAuto(false);
        m_poseEstimator.stopNoteTracking();
    });

    private final SmartShootByPose m_shoot = new SmartShootByPose(m_shooter, m_drive, m_pitcher, m_driverController,
                        m_poseEstimator::getPose);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureAutoBuilder();
        configureNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser();

        m_drive.setDefaultCommand(m_driveByController);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        // Configure the trigger bindings
        configureBindings();

    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {

        m_demoController.leftTrigger(0.25)
                .onTrue(new InstantCommand(() -> m_pitcher.pitchToAngle(16.0))
                        .alongWith(new InstantCommand(() -> m_shooter.run(80.0, 25.0))))
                .onFalse(new InstantCommand(() -> m_pitcher.pitchToAngle(PitcherConstants.kHome))
                        .alongWith(new InstantCommand(() -> m_shooter.stop())));

        m_demoController.leftBumper()
                .onTrue(new InstantCommand(() -> m_pitcher.pitchToAngle(12.0))
                        .alongWith(new InstantCommand(() -> m_shooter.run(40.0, 25.0))))
                .onFalse(new InstantCommand(() -> m_pitcher.pitchToAngle(PitcherConstants.kHome))
                        .alongWith(new InstantCommand(() -> m_shooter.stop())));

                        m_demoController.rightBumper().onTrue(new InstantCommand(() ->{
m_shooter.run(-10.0, 0.0);
m_feeder.run(-0.2);
                        })).onFalse(new InstantCommand(()->{
                                m_shooter.stop();
                                m_feeder.stop();
                        }));

        m_demoController.rightTrigger(0.25)
                // .whileTrue(new BiDirectionalIntake(m_intaker, m_drive, m_indexer, m_feeder,
                // m_demoController))
                // .onFalse(new ReverseFeed(m_feeder, m_indexer).withTimeout(0.130));
                .onTrue(new InstantCommand(() -> m_feeder.run(0.6))
                        .alongWith(new InstantCommand(() -> m_indexer.run(0.6))))
                .onFalse(new InstantCommand(() -> m_feeder.stop())
                        .alongWith(new InstantCommand(() -> m_indexer.stop())));

        m_demoController.y().onTrue(m_pitcher.changePitch(2.0));
        m_demoController.b().onTrue(m_pitcher.changePitch(-2.0));

        m_demoController.pov(0).onTrue(m_shooter.changeSpeed(10.0));
        m_demoController.pov(180).onTrue(m_shooter.changeSpeed(-10.0));

        m_driverController.pov(0).onTrue(
                new InstantCommand(
                        () -> {
                            var alliance = DriverStation.getAlliance();
                            if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                                m_poseEstimator
                                        .resetOdometry(new Pose2d(new Translation2d(15.17, 5.55), new Rotation2d()));
                            } else {
                                m_poseEstimator.resetOdometry(
                                        new Pose2d(new Translation2d(1.31, 5.55), new Rotation2d(Math.PI)));
                            }
                        }));
        m_driverController.pov(180).onTrue(new InstantCommand(
                () -> m_drive.resetOdometry(new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(Math.PI)))));

        m_driverController.leftTrigger(0.25)
                .whileTrue(m_shoot)
                .onTrue(switchLimelightPipeline("limelight-front", 1)
                        .alongWith(new InstantCommand(() -> m_poseEstimator.setAuto(false))))
                .onFalse(switchLimelightPipeline("limelight-front", 0));

        m_driverController.rightTrigger(0.25).whileTrue(new Feed(m_feeder, m_indexer, m_intaker));
                // .onTrue(new InstantCommand(() -> m_feeder.run(0.6))
                //         .alongWith(new InstantCommand(() -> m_indexer.run(0.6))))
                // .onFalse(new InstantCommand(() -> m_feeder.stop())
                //         .alongWith(new InstantCommand(() -> m_indexer.stop())));

        m_driverController.leftBumper()
                .whileTrue(new BiDirectionalIntake(m_intaker, m_drive, m_indexer, m_feeder, m_driverController))
                .onFalse(new ReverseIntake(m_feeder, m_indexer, m_intaker).withTimeout(0.130));
        // .onFalse(new InstantCommand(() -> m_feeder.run(-0.4)).andThen(new
        // WaitCommand(0.06))
        // .andThen(new InstantCommand(() -> m_feeder.stop()).alongWith(new
        // InstantCommand(() -> m_indexer.stop()))));

        m_driverController.rightBumper()
                .whileTrue(
                        new Handoff(m_intaker, m_indexer, m_manipulator, m_feeder, -14.0, m_elevator, m_drive, false));

        m_driverController.b().onTrue(new InstantCommand(() -> m_manipulator.run(.4)))
                .onFalse(new InstantCommand(() -> m_manipulator.stop()));

        m_driverController.start().onTrue(new InstantCommand(() -> m_climber.setPose(0.0)));
        m_driverController.back()
                .onTrue(new InstantCommand(() -> m_climber.setPose(68.0)).alongWith(new ZeroElevator(m_elevator)));

        m_driverController.pov(90).onTrue(new InstantCommand(() -> m_manipulator.run(-0.2)))
                .onFalse(new InstantCommand(() -> m_manipulator.stop()));

        m_driverController.pov(270)
                .onTrue(new Handoff(m_intaker, m_indexer, m_manipulator, m_feeder, -19.8, m_elevator, m_drive, false)
                        .withTimeout(2.0));

        m_driverController.x().onTrue(new InstantCommand(()->{
                                m_poseEstimator.trackNote(2);
                        })).onFalse(new InstantCommand(()->{
                                m_poseEstimator.stopNoteTracking();
                        }));

        m_driverController.a().onTrue(new InstantCommand(() -> m_elevator.setPose(16.0)))
                .onFalse(new InstantCommand(() -> m_elevator.setPose(1.0)).andThen(new WaitCommand(0.25))
                        .andThen(new ZeroElevator(m_elevator)));
        m_driverController.y().onTrue(new InstantCommand(() -> m_elevator.setPose(24.5)))
                .onFalse(new InstantCommand(() -> m_elevator.setPose(ElevatorConstants.kRest)));

        m_operatorPanel.button(2)
                .whileTrue(new AutoIntakeAimAssist(m_drive, 1.5)
                        .raceWith(new WaitCommand(0.1)
                                .andThen(new AutoIntake(m_intaker, m_indexer, m_feeder, true, m_shooter))
                                .alongWith(new InstantCommand(() -> m_elevator.setPose(1.0)))
                                .alongWith(switchLimelightPipeline("limelight-note", 0)))
                        .andThen(new ReverseIntake(m_feeder, m_indexer, m_intaker).withTimeout(0.150)))
                .onFalse(new InstantCommand(() -> m_elevator.setPose(1.0))
                        .alongWith(new PerpetualIntake(m_intaker, m_indexer, m_feeder, -1.0).withTimeout(.25)));
        m_operatorPanel.button(3).onTrue(

                new InstantCommand(() -> {
                    m_intaker.runIndividual(-.2 * .7, -.2);
                    m_feeder.run(-0.4);
                    m_indexer.run(-.4);
                })).onFalse(
                        new InstantCommand(() -> {
                            m_intaker.stop();
                            m_feeder.stop();
                            m_indexer.stop();
                        }));

        m_operatorPanel.button(1).whileTrue(new ZeroPitcher(m_pitcher));

        m_operatorPanel.button(4).onTrue(

                new InstantCommand(() -> {
                    m_intaker.runIndividual(.6 * .7, -.6);
                    m_feeder.run(-0.6);
                    m_indexer.run(-.6);
                })).onFalse(
                        new InstantCommand(() -> {
                            m_intaker.stop();
                            m_feeder.stop();
                            m_indexer.stop();
                        }));

    }

    public void configureNamedCommands() {
        PerpetualIntake perpetualIntakeBack = new PerpetualIntake(m_intaker, m_indexer, m_feeder, -1.0);
        CheckForNote checkBNote = new CheckForNote();

        NamedCommands.registerCommand("PathDecider",
                new ConditionalCommand(new WaitCommand(15.0), new WaitCommand(0.0),
                        m_feeder::getProx));
        NamedCommands.registerCommand("TelePose", new InstantCommand(() -> m_poseEstimator.setAuto(false)));
        NamedCommands.registerCommand("AutoPose", new InstantCommand(() -> m_poseEstimator.setAuto(true)));
        NamedCommands.registerCommand("Auto Shooter By Pose",
                new AutoShooterByPose(m_shooter, m_drive, m_pitcher, m_poseEstimator::getPose));
        NamedCommands.registerCommand("PathDeciderB",
                new ConditionalCommand(new WaitCommand(15.0), new WaitCommand(0.0), checkBNote::sawNote));
        NamedCommands.registerCommand("CheckForB", checkBNote);
        NamedCommands.registerCommand("Perpetual Intake Decider", perpetualIntakeBack);
        NamedCommands.registerCommand("Perpetual Intake Back",
                new PerpetualIntake(m_intaker, m_indexer, m_feeder, -1.0));
        NamedCommands.registerCommand("Perpetual Intake Front",
                new PerpetualIntake(m_intaker, m_indexer, m_feeder, 1.0));
        NamedCommands.registerCommand("Limelight Pipeline 1", switchLimelightPipeline("limelight-front", 1)
                .alongWith(new InstantCommand(() -> m_poseEstimator.setAuto(false))));
        NamedCommands.registerCommand("Limelight Pipeline 0", switchLimelightPipeline("limelight-front", 0)
                .alongWith(new InstantCommand(() -> m_poseEstimator.setAuto(true))));
        NamedCommands.registerCommand("Stop Drive", new InstantCommand(() -> m_drive.stop()));
        NamedCommands.registerCommand("Aim Shooter",
                new SmartShootByPose(m_shooter, m_drive, m_pitcher, m_driverController, m_poseEstimator::getPose));
        NamedCommands.registerCommand("Soft Shoot", new InstantCommand(() -> {
            m_shooter.run(12.5);

        }));
        NamedCommands.registerCommand("UpdatePose", new InstantCommand(() -> m_poseEstimator.updatePose()));
        NamedCommands.registerCommand("NoteFar", switchLimelightPipeline("limelight-note", 0));
        NamedCommands.registerCommand("NoteClose", switchLimelightPipeline("limelight-note", 1));
        NamedCommands.registerCommand("AimAtCenter", new RotateToAngle(new Rotation2d(-Math.PI / 2.0), m_drive));
        NamedCommands.registerCommand("AimAtNote", new RotateToAngle(new Rotation2d(Math.PI / 4.0), m_drive));
        NamedCommands.registerCommand("Step Back",
                new AutoMoveNShoot(m_shooter, m_drive, m_pitcher, m_poseEstimator::getPose, 2.3, 0.0));
        NamedCommands.registerCommand("Step Back Slow",
                new AutoMoveNShoot(m_shooter, m_drive, m_pitcher, m_poseEstimator::getPose, 2.0, 0.0));
        NamedCommands.registerCommand("Step Back Slower",
                new AutoMoveNShoot(m_shooter, m_drive, m_pitcher, m_poseEstimator::getPose, 1.7, 0.0));
        NamedCommands.registerCommand("Kick Back",
                new ReverseIntake(m_feeder, m_indexer, m_intaker).withTimeout(0.170));
        NamedCommands.registerCommand("Stop Shooter", new InstantCommand(() -> m_shooter.stop()));
        NamedCommands.registerCommand("DropNoteFront", new MoveToFloor(m_feeder, m_indexer, m_intaker, false));

        NamedCommands.registerCommand("TrackNoteA", new InstantCommand(() -> {
            m_poseEstimator.trackNote(4);
        }));
        NamedCommands.registerCommand("TrackNoteB", new InstantCommand(() -> {
            m_poseEstimator.trackNote(3);
        }));
        NamedCommands.registerCommand("TrackNoteC", new InstantCommand(() -> {
            m_poseEstimator.trackNote(2);
        }));
        NamedCommands.registerCommand("TrackNoteD", new InstantCommand(() -> {
            m_poseEstimator.trackNote(1);
        }));
        NamedCommands.registerCommand("TrackNoteE", new InstantCommand(() -> {
            m_poseEstimator.trackNote(0);
        }));
        NamedCommands.registerCommand("TrackSneakAmp", new InstantCommand(() -> {
            m_poseEstimator.trackSneakNote(5);
        }));
        NamedCommands.registerCommand("TrackSneakSource", new InstantCommand(() -> {
            m_poseEstimator.trackSneakNote(6);
        }));
        NamedCommands.registerCommand("StopNoteTracking", new InstantCommand(() -> {
            m_poseEstimator.stopNoteTracking();
        }));

    }

    public void configureAutoBuilder() {
        AutoBuilder.configureHolonomic(m_poseEstimator::getPose, m_poseEstimator::resetOdometry,
                m_drive::getChassisSpeed,
                m_drive::drive, Auto.autoConfig,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, m_drive);
    }

    /**
     *
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();

    }

    public Command getTestCommand() {
        return new ZeroClimber(m_climber).alongWith(new ZeroElevator(m_elevator));
    }

    public Command getTeleInitCommand() {
        return m_teleInitCommand;
    }

    public Command switchLimelightPipeline(String name, int pipeline) {
        return new InstantCommand(() -> LimelightHelpers.setPipelineIndex(name, pipeline));
    }

}
