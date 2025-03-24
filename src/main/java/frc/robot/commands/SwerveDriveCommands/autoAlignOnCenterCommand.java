package frc.robot.commands.SwerveDriveCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsytems.SwerveDriveManager;
public class autoAlignOnCenterCommand extends Command {
    private final SwerveDriveManager driveMananger;
    public autoAlignOnCenterCommand(SwerveDriveManager driveMananger) { 
        this.driveMananger = driveMananger;
        addRequirements(driveMananger);
    }

    @Override
    public void initialize(){
        driveMananger.alignCenter();
        System.out.println("Aligning On center");
        SmartDashboard.putString("Align on : ", "CENTER");
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
