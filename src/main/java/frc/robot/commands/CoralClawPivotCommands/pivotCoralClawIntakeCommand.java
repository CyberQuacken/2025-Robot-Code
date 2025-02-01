package frc.robot.commands.CoralClawPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.coralClawConstants;
import frc.robot.subsystems.CoralClawSubsystems.CoralClawPivotSubsystem;

public class pivotCoralClawIntakeCommand extends Command{
    /**
     * <The default command>
     * to choose where it moves to use the set commands
     */
    CoralClawPivotSubsystem m_coralClawPivotSubsystem;

    public pivotCoralClawIntakeCommand(CoralClawPivotSubsystem subsystem){
        m_coralClawPivotSubsystem = subsystem;
        addRequirements(m_coralClawPivotSubsystem);
    }

    @Override
    public void initialize(){
        // <Should outtake speed be from constants or container?>
        m_coralClawPivotSubsystem.setCurrentPosition(coralClawConstants.intakePositionIndex);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
