package frc.robot.commands.AlgeaCommands.algeaScrubberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgeaSubsytems.AlgeaScrubberPivotSubsytem;

public class moveScrubberIn extends Command{
    // will move the motor out to a suitable Position
    AlgeaScrubberPivotSubsytem scrubberPivot;

    public moveScrubberIn (AlgeaScrubberPivotSubsytem subsytem){
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
