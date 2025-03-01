package frc.robot.commands.AlgaeScrubberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsytems.AlgaeScrubberPivotSubsytem;

public class moveScrubberOut extends Command{
    // will move the motor out to a suitable Position
    AlgaeScrubberPivotSubsytem scrubberPivot;

    public moveScrubberOut (AlgaeScrubberPivotSubsytem subsytem){
        scrubberPivot = subsytem;

        addRequirements(scrubberPivot);
    }

    @Override
    public void execute(){
        scrubberPivot.moveMotorOut();
    }

    @Override
    public void end(boolean interrupted){
        scrubberPivot.stopMotor();
    }
}
