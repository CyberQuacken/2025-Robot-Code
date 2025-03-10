package frc.robot.commands.algaeHarvesterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.algaeHarvesterConstants;
import frc.robot.subsystems.AlgaeSubsytems.Harvester.algaeHarvesterPivot;
public class algaeHarvesterPivotDownCommand extends Command {
    private static algaeHarvesterPivot m_subsystem;
    public algaeHarvesterPivotDownCommand(algaeHarvesterPivot m_algaeHarvesterPivot){
        m_subsystem = m_algaeHarvesterPivot;
        addRequirements(m_algaeHarvesterPivot);
    }
    @Override
    public void initialize(){
        // <Should outtake speed be from constants or container?>
        if(m_subsystem.getCurrentPosition() > algaeHarvesterConstants.horizontalIndex) { 
            m_subsystem.setCurrentPosition(m_subsystem.getCurrentPosition()-1);
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}



