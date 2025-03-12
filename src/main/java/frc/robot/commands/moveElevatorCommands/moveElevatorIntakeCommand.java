package frc.robot.commands.moveElevatorCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.elevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class moveElevatorIntakeCommand extends Command {
    private static ElevatorSubsystem m_subsystem;
    private static double oldPosition = 0.0;

    public moveElevatorIntakeCommand(ElevatorSubsystem subsystem) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    oldPosition = m_subsystem.getAveragePosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.moveElevator(-.1);
    //oldPosition = m_subsystem.getAveragePosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("oldPosition", oldPosition);
    SmartDashboard.putNumber("currentPosition", m_subsystem.getAveragePosition());
    SmartDashboard.putNumber("difference", Math.abs(oldPosition - m_subsystem.getAveragePosition()));
    return false;
  }
}
