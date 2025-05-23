/* 
*
*    This is just a command that I used to test if the visionsubsytem could work. 
*    We can delete this oncevision is done. 
*
*
*/


package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DriveSubsytems.SwerveDriveManager;

public class TestLightCommand extends Command {
    private final VisionSubsystem visionSubsystem;
    private final SwerveDriveManager driveMananger;
    public TestLightCommand(VisionSubsystem visionSubsystem, SwerveDriveManager driveMananger) { 
        this.visionSubsystem = visionSubsystem;
        this.driveMananger = driveMananger;
        addRequirements(visionSubsystem, driveMananger);
    }
    @Override
    public void execute() { 
        if(visionSubsystem.getDetection()) { 
            driveMananger.driveSystem.moveRot(visionSubsystem.getOffset(), true);
        } else { 
        }
    }
    
}
