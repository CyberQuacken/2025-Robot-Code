package frc.robot.commands.moveElevatorCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.elevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class moveElevator extends Command {
    private final ElevatorSubsystem m_Subsystem;
    private final int level;

    public moveElevator(ElevatorSubsystem subsystem, int desiredLevel){
        m_Subsystem = subsystem;
        level = desiredLevel;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // if position is available above current possition and isnt intake, then add
        m_Subsystem.setCurrentPosition(level);
        System.out.println("Elevator to " + level);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        /* 
        // for hold down
        if (m_Subsystem.getCurrentPosition() < elevatorConstants.levelFourPositionIndex){ // 4 is highest scoring level
            m_Subsystem.setCurrentPosition(m_Subsystem.getCurrentPosition()+1);
        }
        System.out.println("ElevatorUp");
        */
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
