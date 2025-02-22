package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveMananger;

public class toggleAlignment extends Command{
    
    private SwerveDriveMananger swerveMananger;

    public toggleAlignment(SwerveDriveMananger swerveMananger) {
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
