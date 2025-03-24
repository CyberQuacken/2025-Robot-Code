package frc.robot.commands.SwerveDriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsytems.SwerveDriveManager;

public class reduceSpeedCommand extends Command{
    
    private SwerveDriveManager swerveMananger;

    @Override
    public void initialize(){
        RobotContainer.reducedSpeed = true;
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
