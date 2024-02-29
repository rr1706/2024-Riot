// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoShooter;
import frc.robot.commands.BiDirectionalIntake;
import frc.robot.commands.DriveByController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Handoff;
import frc.robot.commands.SmartShooter;
import frc.robot.commands.ZeroClimber;
import frc.robot.commands.ZeroElevator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pitcher;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

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

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final DriveByController m_driveByController = new DriveByController(m_drive, m_driverController);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    m_drive.setDefaultCommand(m_driveByController);
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    

    m_driverController.pov(0).onTrue(new InstantCommand(() -> m_drive.resetOdometry(new Pose2d())));
    m_driverController.pov(180).onTrue(new InstantCommand(() -> m_drive.resetOdometry(new Rotation2d(Math.PI))));

    m_driverController.leftTrigger(0.25).whileTrue(new SmartShooter(m_shooter, m_drive, m_pitcher, m_driverController));

    //m_driverController.leftTrigger(0.25).onTrue(new InstantCommand(()->m_shooter.run(90)).alongWith(new InstantCommand(()->m_pitcher.pitchToAngle(6.5)))).onFalse(new InstantCommand(()->m_shooter.stop()).alongWith(new InstantCommand(()->m_pitcher.pitchToAngle(2.0))));
    m_driverController.rightTrigger(0.25).onTrue(new InstantCommand(()->m_feeder.run(0.4)).alongWith(new InstantCommand(()->m_indexer.run(0.4)))).onFalse(new InstantCommand(()->m_feeder.stop()).alongWith(new InstantCommand(()->m_indexer.stop())));
    
    m_driverController.leftBumper().whileTrue(new BiDirectionalIntake(m_intaker, m_drive, m_indexer, m_feeder)).onFalse(new InstantCommand(()->m_feeder.run(-0.4)).andThen(new WaitCommand(0.06)).andThen(new InstantCommand(()->m_feeder.stop()).alongWith(new InstantCommand(()->m_indexer.stop()))));
    //m_driverController.leftBumper().onTrue(new InstantCommand(()->m_intaker.run(1.0,m_drive.getChassisSpeed().vxMetersPerSecond)).alongWith(new InstantCommand(()->m_indexer.run(1.0))).alongWith(new InstantCommand(()->m_feeder.run(0.4))))
      //.onFalse(new InstantCommand(()->m_intaker.stop()).alongWith(new InstantCommand(()->m_indexer.stop())).alongWith(new InstantCommand(()->m_feeder.stop())));
    
      //m_driverController.rightBumper().onTrue(new InstantCommand(()->m_feeder.run(-1.0)).alongWith(new InstantCommand(()->m_indexer.run(-0.3))).alongWith(new InstantCommand(()->m_intaker.run(1.0))).alongWith(new InstantCommand(()->m_manipulator.set(-.4)))).onFalse(new InstantCommand(()->m_feeder.stop()).alongWith(new InstantCommand(()->m_indexer.stop())).alongWith(new InstantCommand(()->m_intaker.stop())).alongWith(new InstantCommand(()->m_manipulator.stop())));
    m_driverController.rightBumper().whileTrue(new Handoff(m_intaker, m_indexer, m_manipulator, m_feeder, -17.0).alongWith(new InstantCommand(()->m_elevator.setPose(3.0))));

    m_driverController.b().onTrue(new InstantCommand(()->m_manipulator.run(.4))).onFalse(new InstantCommand(()->m_manipulator.stop()));
    //m_driverController.y().onTrue(new InstantCommand(()->m_manipulator.set(-.4))).onFalse(new InstantCommand(()->m_manipulator.stop()));
  
    m_driverController.start().onTrue(new InstantCommand(()->m_climber.setPose(0.0)));
    m_driverController.back().onTrue(new InstantCommand(()->m_climber.setPose(68.0)).alongWith(new ZeroElevator(m_elevator)));

    m_driverController.pov(270).onTrue(new Handoff(m_intaker, m_indexer, m_manipulator, m_feeder, -20.0).withTimeout(2.0));

    m_driverController.a().onTrue(new InstantCommand(()->m_elevator.setPose(16.0))).onFalse(new InstantCommand(()->m_elevator.setPose(2.5)).andThen(new WaitCommand(0.5)).andThen(new ZeroElevator(m_elevator)));
    m_driverController.y().onTrue(new InstantCommand(()->m_elevator.setPose(24.5))).onFalse(new InstantCommand(()->m_elevator.setPose(2.5)));

    }

    public void configureNamedCommands(){
      NamedCommands.registerCommand("Run Shooter", new AutoShooter(m_shooter, m_pitcher, m_drive));
      NamedCommands.registerCommand("Run Intake", new BiDirectionalIntake(m_intaker, m_drive, m_indexer, m_feeder));
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

  public Command getTestCommand(){
    return new ZeroClimber(m_climber).alongWith(new ZeroElevator(m_elevator));
  }
}
