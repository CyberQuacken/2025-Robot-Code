package frc.robot.commands.SwerveDriveCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsytems.SwerveDriveManager;
public class autoAlignOnLeftCommand extends Command {
    private final SwerveDriveManager driveMananger;
    public autoAlignOnLeftCommand(SwerveDriveManager driveMananger) { 
        this.driveMananger = driveMananger;
        addRequirements(driveMananger);
    }

    @Override
    public void initialize(){
        driveMananger.alignLeft();
        System.out.println("Aligning On Left");
        SmartDashboard.putString("Align on : ", "LEFT");
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
