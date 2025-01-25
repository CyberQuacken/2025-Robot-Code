/* 
*
*    This is just a command that I used to test if the visionsubsytem could work. 
*    We can delete this oncevision is done. 
*
*
*/


package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.VisionSubsystem;

public class TestLightCommand extends Command {
    private final VisionSubsystem visionSubsystem;
    private final SwerveDrive swerveDrive;
    public TestLightCommand(VisionSubsystem visionSubsystem, SwerveDrive swerveDrive) { 
        this.visionSubsystem = visionSubsystem;
        this.swerveDrive = swerveDrive;
        addRequirements(visionSubsystem, swerveDrive);
    }
    @Override
    public void execute() { 
        if(visionSubsystem.getDetection()) { 
            System.out.println("april");
            swerveDrive.moveRot(visionSubsystem.getOffset(), true);
        } else { 
            System.out.println("NO!");
        }
    }
    
}
