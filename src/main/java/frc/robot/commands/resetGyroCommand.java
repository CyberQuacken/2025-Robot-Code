package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
public class resetGyroCommand extends Command {
    private final SwerveDrive swerveDrive;
    public resetGyroCommand(SwerveDrive swerveDrive) { 
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize(){
        swerveDrive.zeroHeading();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
