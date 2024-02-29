package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pitcher;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.MathUtils;

public class AimShooter extends Command {
    private final Shooter m_shooter;
    private final Drivetrain m_robotDrive;
    private final Pitcher m_pitcher;
    private final PIDController  m_pid = new PIDController(0.1,0.0,0.0);
    private final InterpolatingDoubleTreeMap m_pitchTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_velocityTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_distTable = new InterpolatingDoubleTreeMap();
    private final CommandXboxController m_controller;
    private double manualHoodValue = 5.0;
    private boolean manualHoodOverride = false;
    private double manualVelocityValue = 70.0;
    private boolean manualVelocityOverride = false;
    public AimShooter(Shooter shooter, Drivetrain robotDrive, Pitcher pitcher, CommandXboxController controller){
        m_shooter = shooter;
        m_robotDrive = robotDrive;
        m_pitcher = pitcher;
        m_controller = controller;

        m_distTable.put(16.75,38.0);
        m_distTable.put(12.15, 52.25);
        m_distTable.put(8.95, 56.0);
        m_distTable.put(6.11, 44.625+38.0);
        m_distTable.put(3.76, 66.0+38.0);
        m_distTable.put(2.17, 82.0+38.0);
        m_distTable.put(0.23, 107.0+38.0);
        m_distTable.put(-1.33, 128.5+38.0);
        m_distTable.put(-2.49, 150.5+38.0);
        m_distTable.put(-3.57, 174.0+38.0);
        m_distTable.put(-3.99, 196.0+38.0);
        m_distTable.put(-4.7, 221.5+38.0);

        m_pitchTable.put(41.0,20.0);
        m_pitchTable.put(53.68,18.50);
        m_pitchTable.put(63.77,16.67);
        m_pitchTable.put(85.77,14.97);
        m_pitchTable.put(100.2,13.41);
        m_pitchTable.put(120.0,12.40);
        m_pitchTable.put(140.0,10.54);
        m_pitchTable.put(160.5,9.05);
        m_pitchTable.put(171.0,7.86);
        m_pitchTable.put(179.0,7.27);
        m_pitchTable.put(200.0,6.51);

        m_velocityTable.put(41.0,47.0);
        m_velocityTable.put(53.68,47.0);
        m_velocityTable.put(63.77,47.0);
        m_velocityTable.put(85.77,47.0);
        m_velocityTable.put(100.2,51.33);
        m_velocityTable.put(120.0,53.25);
        m_velocityTable.put(140.0,56.03);
        m_velocityTable.put(160.5,60.09);
        m_velocityTable.put(171.0,70.13);
        m_velocityTable.put(179.0,80.54);
        m_velocityTable.put(200.0,85.6);
        
        addRequirements(m_robotDrive);

    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        SmartDashboard.putBoolean("Manual Velocity Override", manualVelocityOverride);
        SmartDashboard.putNumber("Set Velocity Adjust", manualVelocityValue);

        SmartDashboard.putBoolean("Manual Hood Override", manualHoodOverride);
        SmartDashboard.putNumber("Set Hood Adjust", manualHoodValue);
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        double tx = LimelightHelpers.getTX("limelight-april");

        double distance = m_distTable.get(LimelightHelpers.getTY("limelight-april"));

        SmartDashboard.putNumber("Distance", distance);

        boolean tv = LimelightHelpers.getTV("limelight-april");
        double desiredRot = -MathUtils.inputTransform(m_controller.getRightX())* DriveConstants.kMaxAngularSpeed;

        if(tv){
            desiredRot = m_pid.calculate(tx);
        }

        manualHoodOverride = SmartDashboard.getBoolean("Manual Hood Override", false);
        manualVelocityOverride = SmartDashboard.getBoolean("Manual Velocity Override", false);

        if(manualHoodOverride && manualVelocityOverride){
            manualHoodValue = SmartDashboard.getNumber("Set Hood Adjust", 0);
            manualVelocityValue = SmartDashboard.getNumber("Set Velocity Adjust", 70.0);
            m_pitcher.pitchToAngle(manualHoodValue);
            m_shooter.run(manualVelocityValue);
        }
        else if(manualHoodOverride){
            manualHoodValue = SmartDashboard.getNumber("Set Hood Adjust", 0);
            m_pitcher.pitchToAngle(manualHoodValue);
        }
        else if(manualVelocityOverride){
            manualVelocityValue = SmartDashboard.getNumber("Set Velocity Adjust", 70.0);
            m_shooter.run(manualVelocityValue);
        }
        else{
            m_pitcher.pitchToAngle(m_pitchTable.get(distance));
            m_shooter.run(m_velocityTable.get(distance));
        }

        
        
        
        double desiredTrans[] = MathUtils.inputTransform(-m_controller.getLeftY(), -m_controller.getLeftX());
        double maxLinear = DriveConstants.kMaxSpeedMetersPerSecond;

        desiredTrans[0] *= maxLinear;
        desiredTrans[1] *= maxLinear;

        //double desiredRot = -MathUtils.inputTransform(m_controller.getRightX())* DriveConstants.kMaxAngularSpeed;

        //Translation2d rotAdj= desiredTranslation.rotateBy(new Rotation2d(-Math.PI/2.0)).times(desiredRot*0.05);

        //desiredTranslation = desiredTranslation.plus(rotAdj);

        m_robotDrive.drive(desiredTrans[0], desiredTrans[1],desiredRot,true,true);
        

/*     m_robotDrive.drive(m_slewX.calculate(
        -inputTransform(m_controller.getLeftY()))
        * DriveConstants.kMaxSpeedMetersPerSecond,
        m_slewY.calculate(
            -inputTransform(m_controller.getLeftX()))
            * DriveConstants.kMaxSpeedMetersPerSecond,
        m_slewRot.calculate(-inputTransform(m_controller.getRightX()))
            * DriveConstants.kMaxAngularSpeed,
        fieldOrient); */

        SmartDashboard.putBoolean("DrivingByController", true);
    }
    
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        m_shooter.stop();
        m_pitcher.pitchToAngle(2.0);
    }

    


}
