package frc.robot.commands.SwerveDriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
public class DriveFieldCommand extends Command {
    
    @Override
    public void execute(){
        RobotContainer.fieldCentric = true;
    }
    @Override
    public boolean isFinished(){
        return true;
    }
}
