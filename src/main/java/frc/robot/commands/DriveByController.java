package frc.robot.commands;

import frc.robot.Constants.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.MathUtils;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Implements a DriveByController command which extends the Command class
 */
public class DriveByController extends Command {
  private final Drivetrain m_robotDrive;
  private final CommandXboxController m_controller;

  private boolean fieldOrient = true;

  /**
   * Contructs a DriveByController object which applys the driver inputs from the
   * controller to the swerve drivetrain
   * 
   * @param drive      is the swerve drivetrain object which should be created in
   *                   the RobotContainer class
   * @param controller is the user input controller object for controlling the
   *                   drivetrain
   */
  public DriveByController(Drivetrain drive, CommandXboxController controller) {
    m_robotDrive = drive; // Set the private member to the input drivetrain
    m_controller = controller; // Set the private member to the input controller
    addRequirements(m_robotDrive); // Because this will be used as a default command, add the subsystem which will
                                   // use this as the default
  }

  /**
   * the execute function is overloaded with the function to drive the swerve
   * drivetrain
   */
  @Override
  public void execute() {
    var alliance = DriverStation.getAlliance();

    double xInput = -m_controller.getLeftY();
    double yInput = -m_controller.getLeftX();

    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      xInput = -xInput;
      yInput = -yInput;
    }

    double desiredTrans[] = MathUtils.inputTransform(xInput, yInput);
    double maxLinear = DriveConstants.kMaxSpeedMetersPerSecond;

    desiredTrans[0] *= maxLinear;
    desiredTrans[1] *= maxLinear;

    double desiredRot = -MathUtils.inputTransform(m_controller.getRightX()) * DriveConstants.kMaxAngularSpeed;

    m_robotDrive.drive(desiredTrans[0], desiredTrans[1], desiredRot, true, true);

  }

  @Override
  public void end(boolean interrupted) {

  }

  /**
   * when this fucntion of the command is called the current fieldOrient boolean
   * is flipped. This
   * is fed into the drive command for the swerve drivetrain so the driver can
   * decide to drive in
   * a robot oreinted when they please (not recommended in most instances)
   */
  public void changeFieldOrient() {
    if (fieldOrient) {
      fieldOrient = false;
    } else {
      fieldOrient = true;
    }
  }

}
