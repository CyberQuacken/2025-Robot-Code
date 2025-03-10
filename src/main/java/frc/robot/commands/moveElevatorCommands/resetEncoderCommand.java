package frc.robot.commands.moveElevatorCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.elevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class resetEncoderCommand extends Command {
    private final ElevatorSubsystem m_Subsystem;
    //private final Timer timer;

    public resetEncoderCommand(ElevatorSubsystem subsystem){
        m_Subsystem = subsystem;
        addRequirements(subsystem);
        //timer = new Timer(); // want to make a version where you hold down instead of press multiple times
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_Subsystem.resetEncoder();
    }

    // Called every time the scheduler runs while the command is scheduled.

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
