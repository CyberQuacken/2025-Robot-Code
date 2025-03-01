package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsytems.SwerveDriveMananger;

public class toggleLimelightAuto extends Command{
    
    private SwerveDriveMananger swerveMananger;

    public toggleLimelightAuto(SwerveDriveMananger swerveMananger) {
        this.swerveMananger = swerveMananger;
        addRequirements(swerveMananger);
    }

    @Override
    public void initialize(){
        swerveMananger.toggleLimelightAuto();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
