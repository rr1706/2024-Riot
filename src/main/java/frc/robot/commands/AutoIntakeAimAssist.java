package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.MathUtils;

public class AutoIntakeAimAssist extends Command {
    private final Drivetrain m_drive;
    private final PIDController m_pid = new PIDController(0.075, 0.025, 0.0);
    private final SlewRateLimiter m_slewX = new SlewRateLimiter(6.0);
    private final SlewRateLimiter m_slewY = new SlewRateLimiter(6.0);
    private final SlewRateLimiter m_slewRot = new SlewRateLimiter(6.0);
    private boolean m_detectedOnce = false;
    private double ty_check = 25.0;

    public AutoIntakeAimAssist(Drivetrain drive) {
        m_drive = drive;
        m_pid.setIntegratorRange(-0.5, 0.5);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        ty_check = 25.0;
        m_detectedOnce = false;
        m_slewX.reset(m_drive.getChassisSpeed().vxMetersPerSecond);
        m_slewY.reset(m_drive.getChassisSpeed().vyMetersPerSecond);
        m_slewRot.reset(m_drive.getChassisSpeed().omegaRadiansPerSecond);
    }

    @Override
    public void execute() {

        double tx = LimelightHelpers.getTX("limelight-note");
        boolean tv = LimelightHelpers.getTV("limelight-note");
        double ty = LimelightHelpers.getTY("limelight-note");

         double xInput = -2.5;

        if(!tv){
            xInput = -0.3;
        }

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


        double yInput = aimAssist;

        double desiredRot = -aimAssist;




        SmartDashboard.putNumber("TX Note", tx);
        SmartDashboard.putNumber("TY Note", ty);

        // double desiredRot = -MathUtils.inputTransform(m_controller.getRightX())*
        // DriveConstants.kMaxAngularSpeed;

        // Translation2d rotAdj= desiredTranslation.rotateBy(new
        // Rotation2d(-Math.PI/2.0)).times(desiredRot*0.05);

        // desiredTranslation = desiredTranslation.plus(rotAdj);

        m_drive.drive(m_slewX.calculate(xInput), m_slewY.calculate(yInput), m_slewRot.calculate(desiredRot), false, true);

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
