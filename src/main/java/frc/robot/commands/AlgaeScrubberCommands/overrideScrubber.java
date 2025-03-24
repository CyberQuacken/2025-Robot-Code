package frc.robot.commands.AlgaeScrubberCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.algaeScrubberConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.AlgaeSubsytems.Scrubber.AlgaeScrubberPivotSubsytem;
import frc.robot.subsystems.AlgaeSubsytems.Scrubber.AlgaeScrubberSubsystem;

public class overrideScrubber extends Command {
    private final AlgaeScrubberSubsystem m_Subsystem;
    private final AlgaeScrubberPivotSubsytem pivotSubsytem;

    public overrideScrubber(AlgaeScrubberSubsystem subsystem, AlgaeScrubberPivotSubsytem pivotSubsytem){
        m_Subsystem = subsystem;
        this.pivotSubsytem = pivotSubsytem;
        addRequirements(subsystem, pivotSubsytem);
        SmartDashboard.putBoolean("done", true);
    }

    @Override
    public void execute (){
        m_Subsystem.override();
        pivotSubsytem.override();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
