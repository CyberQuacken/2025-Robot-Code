package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.elevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class moveElevatorDownCommand extends Command {
    private final ElevatorSubsystem m_Subsystem;

    public moveElevatorDownCommand(ElevatorSubsystem subsystem){
        m_Subsystem = subsystem;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // if there is a position lower than the current positon. it subtracts to get that position
        if (m_Subsystem.getCurrentPosition() > 0){ // 0 is the lowest position
            m_Subsystem.setCurrentPosition(m_Subsystem.getCurrentPosition()-1);
        }
        else if (m_Subsystem.getCurrentPosition() == elevatorConstants.intakePositionIndex){
            //if current level is equal to 5, change position from intake to level 2
            m_Subsystem.setCurrentPosition(elevatorConstants.levelTwoPositionIndex);// level 2
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
