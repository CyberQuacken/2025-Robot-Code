package frc.robot.commands.algaeHarvesterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsytems.Harvester.algaeHarvesterIntakeSubsystem;

public class algaeHarvesterOuttakeCommand extends Command{

    private static algaeHarvesterIntakeSubsystem m_subsystem;

    public algaeHarvesterOuttakeCommand(algaeHarvesterIntakeSubsystem m_algaeHarvesterIntake){
        m_subsystem = m_algaeHarvesterIntake;
        addRequirements(m_algaeHarvesterIntake);
    }

    @Override
    public void initialize(){
        // <Should outtake speed be from constants or container?> // -- Constants
        m_subsystem.outake();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
