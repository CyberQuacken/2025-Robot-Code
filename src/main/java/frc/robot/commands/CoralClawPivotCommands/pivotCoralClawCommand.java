package frc.robot.commands.CoralClawPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralClawSubsystems.CoralClawPivotSubsystem;

public class pivotCoralClawCommand extends Command{
    /**
     * <The default command>
     * to choose where it moves to use the set commands
     */
    CoralClawPivotSubsystem m_coralClawPivotSubsystem;
    int desiredPosition;

    public pivotCoralClawCommand(CoralClawPivotSubsystem subsystem, int position){
        m_coralClawPivotSubsystem = subsystem;
        addRequirements(m_coralClawPivotSubsystem);
        desiredPosition = position;
    }

    @Override
    public void initialize(){
        // <Should outtake speed be from constants or container?>
        m_coralClawPivotSubsystem.setCurrentPosition(desiredPosition);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
