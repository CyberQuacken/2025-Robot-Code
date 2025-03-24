package frc.robot.commands.moveElevatorCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.elevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class autoL4Command extends Command {
    private final ElevatorSubsystem m_Subsystem;

    public autoL4Command(ElevatorSubsystem subsystem){
        m_Subsystem = subsystem;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_Subsystem.setCurrentPosition(elevatorConstants.levelFourPositionIndex);
        
        System.out.println("ElevatorUp");
    }

    // Called every time the scheduler runs while the command is scheduled.


    // Called once the command ends or is/*  */ interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_Subsystem.isAtDesiredPosition() && m_Subsystem.getCurrentPosition() == elevatorConstants.levelFourPositionIndex;
    }
}
 