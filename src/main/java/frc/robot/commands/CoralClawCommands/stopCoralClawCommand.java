package frc.robot.commands.CoralClawCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralClawSubsystem;

public class stopCoralClawCommand extends Command{
    
    CoralClawSubsystem m_coralClawSubsystem;

    public stopCoralClawCommand(CoralClawSubsystem coralClawSubsystem){
        m_coralClawSubsystem = coralClawSubsystem;
        addRequirements(coralClawSubsystem);
    }

    @Override
    public void initialize(){
        // <Should outtake speed be from constants or container?>
        m_coralClawSubsystem.stopMotors();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
