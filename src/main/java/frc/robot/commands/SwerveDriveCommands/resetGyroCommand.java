package frc.robot.commands.SwerveDriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsytems.SwerveDriveManager;
public class resetGyroCommand extends Command {
    private final SwerveDriveManager driveMananger;
    public resetGyroCommand(SwerveDriveManager driveMananger) { 
        this.driveMananger = driveMananger;
        addRequirements(driveMananger);
    }

    @Override
    public void initialize(){
        driveMananger.driveSystem.zeroHeading();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
