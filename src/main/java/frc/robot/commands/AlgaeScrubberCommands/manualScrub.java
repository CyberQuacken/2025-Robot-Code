package frc.robot.commands.AlgaeScrubberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.algaeScrubberConstants;
import frc.robot.subsystems.AlgaeSubsytems.Scrubber.AlgaeScrubberSubsystem;

public class manualScrub extends Command{
    // will move the motor out to a suitable Position
    AlgaeScrubberSubsystem scrubberPivot;

    public manualScrub (AlgaeScrubberSubsystem subsytem){
        scrubberPivot = subsytem;
        addRequirements(scrubberPivot);
    }

    @Override
    public void execute(){
        scrubberPivot.scrub();
    }

    @Override
    public void end(boolean interrupted){
        scrubberPivot.stopMotor();
    }
}
