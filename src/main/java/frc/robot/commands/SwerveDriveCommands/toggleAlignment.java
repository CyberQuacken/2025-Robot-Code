package frc.robot.commands.SwerveDriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsytems.SwerveDriveManager;

public class toggleAlignment extends Command{
    
    private SwerveDriveManager swerveMananger;

    public toggleAlignment(SwerveDriveManager swerveMananger) {
        this.swerveMananger = swerveMananger;
        addRequirements(swerveMananger);
    }

    @Override
    public void initialize(){
        swerveMananger.toggleAlignment();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
