package frc.robot.commands.AlgeaCommands.algeaScrubberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgeaSubsytems.AlgeaScrubberPivotSubsytem;

public class moveScrubberOut extends Command{
    // will move the motor out to a suitable Position
    AlgeaScrubberPivotSubsytem scrubberPivot;

    public moveScrubberOut (AlgeaScrubberPivotSubsytem subsytem){
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
