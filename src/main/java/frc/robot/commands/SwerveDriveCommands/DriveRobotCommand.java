package frc.robot.commands.SwerveDriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
public class DriveRobotCommand extends Command {
    
    @Override
    public void execute(){
        RobotContainer.fieldCentric = false;
    }
    @Override
    public boolean isFinished(){
        return true;
    }
}
