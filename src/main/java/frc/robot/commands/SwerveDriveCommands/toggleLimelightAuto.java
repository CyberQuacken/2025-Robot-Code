package frc.robot.commands.SwerveDriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

public class toggleLimelightAuto extends Command{
    
    private SwerveDrive swerveDrive;

    public toggleLimelightAuto(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize(){
        swerveDrive.toggleLimelightAuto();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
