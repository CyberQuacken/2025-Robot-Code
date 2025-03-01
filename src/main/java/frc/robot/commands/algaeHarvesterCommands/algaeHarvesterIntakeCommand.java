

package frc.robot.commands.algaeHarvesterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgeaSubsytems.algaeHarvesterIntakeSubsystem;

public class algaeHarvesterIntakeCommand extends Command{

    private static algaeHarvesterIntakeSubsystem m_subsystem;

    public algaeHarvesterIntakeCommand(algaeHarvesterIntakeSubsystem m_algaeHarvesterIntake){
        m_subsystem = m_algaeHarvesterIntake;
        addRequirements(m_algaeHarvesterIntake);
    }

    @Override
    public void initialize(){
        // <Should outtake speed be from constants or container?>
        m_subsystem.intake();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
