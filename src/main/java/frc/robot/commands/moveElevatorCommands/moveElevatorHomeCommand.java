package frc.robot.commands.moveElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.elevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class moveElevatorHomeCommand extends Command{
    private static ElevatorSubsystem m_subsystem;

    public moveElevatorHomeCommand(ElevatorSubsystem subsystem) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setCurrentPosition(elevatorConstants.homePositionIndex);
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
