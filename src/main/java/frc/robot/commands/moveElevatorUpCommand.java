package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class moveElevatorUpCommand extends Command {
    private final ElevatorSubsystem m_Subsystem;

    public moveElevatorUpCommand(ElevatorSubsystem subsystem){
        m_Subsystem = subsystem;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // if there is a position higher than the current positon. it adds to get that position
        if (m_Subsystem.getCurrentPosition() < 5){ // 5 is the top currently
            m_Subsystem.setCurrentPosition(m_Subsystem.getCurrentPosition()+1);
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
