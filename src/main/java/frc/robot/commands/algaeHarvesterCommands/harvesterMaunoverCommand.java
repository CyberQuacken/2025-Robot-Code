package frc.robot.commands.algaeHarvesterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsytems.Harvester.algaeHarvesterIntakeSubsystem;

public class harvesterMaunoverCommand extends Command{

    private static algaeHarvesterIntakeSubsystem m_subsystem;


    public harvesterMaunoverCommand(algaeHarvesterIntakeSubsystem m_algaeHarvesterIntake){
        m_subsystem = m_algaeHarvesterIntake;
        addRequirements(m_algaeHarvesterIntake);
    }
     // Unless you have finished this - DO NOT USE
    @Override
    public void initialize(){
        // <Should outtake speed be from constants or container?> / -- Constants
        if(m_subsystem.getIntake()){ // if robot should intake 
            m_subsystem.intake(); // todo change commands / values
        }
        else{ // already have algae, outaking algae
            m_subsystem.intake(); // todo change commands / values
        }

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
