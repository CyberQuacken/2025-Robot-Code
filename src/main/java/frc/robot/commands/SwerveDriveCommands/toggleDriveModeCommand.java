package frc.robot.commands.SwerveDriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsytems.SwerveDriveManager;
public class toggleDriveModeCommand extends Command {

    @Override
    public void execute(){
        RobotContainer.fieldCentric = !RobotContainer.fieldCentric;
    }
    @Override
    public boolean isFinished(){
        return true;
    }
}
