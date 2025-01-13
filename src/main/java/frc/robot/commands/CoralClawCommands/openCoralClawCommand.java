package frc.robot.commands.CoralClawCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class openCoralClawCommand extends Command{

    CoralClawSubsystem m_coralClawSubsystem;

    public openCoralClawCommand(CoralClawSubsystem coralClawSubsystem){
        m_coralClawSubsystem = coralClawSubsystem;
        addRequirements(coralClawSubsystem);
    }

    @Override
    public void initialize(){
        // <Should outtake speed be from constants or container?>
        m_coralClawSubsystem.outtake(0);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
