package frc.robot.commands.SwerveDriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsytems.SwerveDriveManager;

public class setLimelightAutoOff extends Command{
    
    private SwerveDriveManager swerveMananger;

    public setLimelightAutoOff(SwerveDriveManager swerveMananger) {
        this.swerveMananger = swerveMananger;
        addRequirements(swerveMananger);
    }

    @Override
    public void initialize(){
        swerveMananger.setLimelightAuto(false);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
