package frc.robot.commands.CoralClawCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralClawSubsystems.CoralClawSubsystem;

public class closeCoralClawCommand extends Command{

    CoralClawSubsystem m_coralClawSubsystem;

    public closeCoralClawCommand(CoralClawSubsystem coralClawSubsystem){
        m_coralClawSubsystem = coralClawSubsystem;
        addRequirements(coralClawSubsystem);
    }

    @Override
    public void initialize(){
        // <Should outtake speed be from constants or container?>
        m_coralClawSubsystem.intake(0);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
