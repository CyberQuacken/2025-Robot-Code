package frc.robot.commands.SwerveDriveCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

public class lastRotInput extends Command {
    private final SwerveDrive SwerveDrive;
    private  Rotation2d[] rots;
    private Rotation2d[] prevRots;
    public lastRotInput(SwerveDrive SwerveDrive) { 
        this.SwerveDrive = SwerveDrive;
        rots = SwerveDrive.getLastRots();
        prevRots = rots;
        addRequirements(SwerveDrive);
    }


    @Override
    public void execute() { 
        rots = SwerveDrive.getLastRots();

        if(rots[0] != null && rots[1] != null && rots[2] != null && rots[3] != null){ 
        SwerveDrive.turn(rots);
        }
   

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
