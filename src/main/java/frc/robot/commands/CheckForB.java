package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

public class CheckForB extends Command {
    private boolean sawNote = true;
    public CheckForB(){
    }

    @Override
    public void initialize() {
        sawNote = false;

    }

    @Override
    public void execute() {
        if(LimelightHelpers.getTV("limelight-note")){
            sawNote = true;
        }
        SmartDashboard.putBoolean("SawNote", sawNote);
    }

    public boolean sawNote(){
        return sawNote;
    }

}

