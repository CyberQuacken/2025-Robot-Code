package frc.robot.commands.moveElevatorCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.elevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class overrideElevatorCommand extends Command {
    private final ElevatorSubsystem m_Subsystem;

    public overrideElevatorCommand(ElevatorSubsystem subsystem){
        m_Subsystem = subsystem;
        addRequirements(subsystem);
        SmartDashboard.putBoolean("overriden elevator", true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_Subsystem.override();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
