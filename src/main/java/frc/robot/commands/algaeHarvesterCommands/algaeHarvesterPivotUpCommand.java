package frc.robot.commands.algaeHarvesterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsytems.algaeHarvesterIntakeSubsystem;
import frc.robot.subsystems.AlgaeSubsytems.algaeHarvesterPivot;
public class algaeHarvesterPivotUpCommand extends Command {
    private static algaeHarvesterPivot m_subsystem;
    public algaeHarvesterPivotUpCommand(algaeHarvesterPivot m_algaeHarvesterPivot){
        m_subsystem = m_algaeHarvesterPivot;
        addRequirements(m_algaeHarvesterPivot);
    }
    @Override
    public void initialize(){
        // <Should outtake speed be from constants or container?>
        m_subsystem.pivot(1);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}



