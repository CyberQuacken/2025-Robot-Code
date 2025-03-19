package frc.robot.commands.SwerveDriveCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsytems.SwerveDriveManager;

public class lastRotCommand extends Command {
    private final SwerveDriveManager driveMananger;
    private  Rotation2d[] rots;
    private Rotation2d[] prevRots;
    public lastRotCommand(SwerveDriveManager driveMananger) { 
        this.driveMananger = driveMananger;
        rots = driveMananger.driveSystem.getLastRots();
        prevRots = rots;
        addRequirements(driveMananger);
    }

    @Override
    public void initialize(){}
    @Override
    public void execute() { 
        rots = driveMananger.driveSystem.getLastRots();

        if(rots[0] != null && rots[1] != null && rots[2] != null && rots[3] != null){ 
            driveMananger.driveSystem.turn(rots);
        }
   

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}