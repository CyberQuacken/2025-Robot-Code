package frc.robot.commands.SwerveDriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsytems.SwerveDriveManager;

public class normalSpeedCommand extends Command{
    
    private SwerveDriveManager swerveMananger;

    @Override
    public void initialize(){
        RobotContainer.reducedSpeed = false;
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
