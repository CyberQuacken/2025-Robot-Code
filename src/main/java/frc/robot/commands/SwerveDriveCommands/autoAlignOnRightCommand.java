package frc.robot.commands.SwerveDriveCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsytems.SwerveDriveManager;
public class autoAlignOnRightCommand extends Command {
    private final SwerveDriveManager driveMananger;
    public autoAlignOnRightCommand(SwerveDriveManager driveMananger) { 
        this.driveMananger = driveMananger;
        addRequirements(driveMananger);
    }

    @Override
    public void initialize(){
        driveMananger.alignRight();
        System.out.println("Aligning On Right");
        SmartDashboard.putString("Align on : ", "RIGHT");

        RobotContainer.alignOnLeft = false;
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
