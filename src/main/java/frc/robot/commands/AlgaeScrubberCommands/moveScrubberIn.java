package frc.robot.commands.AlgaeScrubberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsytems.AlgaeScrubberPivotSubsytem;

public class moveScrubberIn extends Command{
    // will move the motor out to a suitable Position
    AlgaeScrubberPivotSubsytem scrubberPivot;

    public moveScrubberIn (AlgaeScrubberPivotSubsytem subsytem){
        scrubberPivot = subsytem;

        addRequirements(scrubberPivot);
    }

    @Override
    public void execute(){
        scrubberPivot.moveMotorIn();
    }

    @Override
    public void end(boolean interrupted){
        scrubberPivot.stopMotor();
    }
}
