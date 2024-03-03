package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.MathUtils;

public class IntakeAimAssist extends Command {
    private final Drivetrain m_drive;
    private final CommandXboxController m_controller;
    private final PIDController m_pid = new PIDController(0.09, 0.0, 0.0);
    private boolean m_detectedOnce = false;
    private double ty_check = 25.0;

    public IntakeAimAssist(Drivetrain drive, CommandXboxController controller) {
        m_drive = drive;
        m_controller = controller;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        ty_check = 25.0;
        m_detectedOnce = false;
    }

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
        double tx = LimelightHelpers.getTX("limelight-note");
        boolean tv = LimelightHelpers.getTV("limelight-note");
        double ty = LimelightHelpers.getTY("limelight-note");

        double desiredRot = -MathUtils.inputTransform(m_controller.getRightX()) * DriveConstants.kMaxAngularSpeed;

        double aimAssist = 0.0;

        if (tv && !m_detectedOnce) {
            m_detectedOnce = true;
            ty_check = ty;
            aimAssist = -1.0*m_pid.calculate(tx);
        } else if (tv && (ty_check + 1.0) >= ty) {
            ty_check = ty;
            aimAssist = -1.0*m_pid.calculate(tx);
        }

        if(aimAssist>=0.5){
            aimAssist = 0.5;
        }
        else if(aimAssist<=-0.5){
            aimAssist = -0.5;
        }

        Translation2d aimAssistAdjusted = new Translation2d(0.0,aimAssist).rotateBy(m_drive.getGyro());

        desiredTrans[0] += aimAssistAdjusted.getX();
        desiredTrans[1] += aimAssistAdjusted.getY();

        SmartDashboard.putNumber("TX Note", tx);
        SmartDashboard.putNumber("TY Note", ty);

        // double desiredRot = -MathUtils.inputTransform(m_controller.getRightX())*
        // DriveConstants.kMaxAngularSpeed;

        // Translation2d rotAdj= desiredTranslation.rotateBy(new
        // Rotation2d(-Math.PI/2.0)).times(desiredRot*0.05);

        // desiredTranslation = desiredTranslation.plus(rotAdj);

        m_drive.drive(desiredTrans[0], desiredTrans[1], desiredRot, true, true);

        /*
         * m_robotDrive.drive(m_slewX.calculate(
         * -inputTransform(m_controller.getLeftY()))
         * DriveConstants.kMaxSpeedMetersPerSecond,
         * m_slewY.calculate(
         * -inputTransform(m_controller.getLeftX()))
         * DriveConstants.kMaxSpeedMetersPerSecond,
         * m_slewRot.calculate(-inputTransform(m_controller.getRightX()))
         * DriveConstants.kMaxAngularSpeed,
         * fieldOrient);
         */

        SmartDashboard.putBoolean("DrivingByController", true);
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

}
