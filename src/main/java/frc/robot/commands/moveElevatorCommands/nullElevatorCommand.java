package frc.robot.commands.moveElevatorCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.algaeScrubberConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.AlgaeSubsytems.Scrubber.AlgaeScrubberPivotSubsytem;
import frc.robot.subsystems.AlgaeSubsytems.Scrubber.AlgaeScrubberSubsystem;

public class nullElevatorCommand extends Command {
    private final ElevatorSubsystem m_Subsystem;

    public nullElevatorCommand(ElevatorSubsystem subsystem){
        m_Subsystem = subsystem;
        addRequirements(subsystem);
        SmartDashboard.putBoolean("done", true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
