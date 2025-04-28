package frc.robot.commands.moveElevatorCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.elevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class runElevator extends Command {
    private final ElevatorSubsystem m_Subsystem;

    public runElevator(ElevatorSubsystem subsystem){
        m_Subsystem = subsystem;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){
        m_Subsystem.moveMotors();
    }


    // Called once the command ends or is/*  */ interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_Subsystem.isAtDesiredPosition(); // AAAAAAAAAAH
    }
}
 