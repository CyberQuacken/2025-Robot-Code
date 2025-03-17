package frc.robot.commands.AlgaeScrubberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.algaeScrubberConstants;
import frc.robot.subsystems.AlgaeSubsytems.Scrubber.AlgaeScrubberPivotSubsytem;
import frc.robot.subsystems.AlgaeSubsytems.Scrubber.AlgaeScrubberSubsystem;

public class ScrubAlgae extends Command{
    // will move the motor out to a suitable Position
    AlgaeScrubberPivotSubsytem scrubberPivot;
    AlgaeScrubberSubsystem scrubber;

    public ScrubAlgae (AlgaeScrubberPivotSubsytem pivot_subsytem, AlgaeScrubberSubsystem subsystem){
        scrubberPivot = pivot_subsytem;
        scrubber = subsystem;

        addRequirements(scrubberPivot, scrubber);
    }

    @Override
    public void execute(){
        scrubberPivot.moveMotorOut();
        if (!(scrubberPivot.getEncoderValue() >= .4)){
            scrubber.scrub();
        }
        // Find a way to turn it off after a certain
    }

    @Override
    public void end(boolean interrupted){
        scrubberPivot.stopMotor();
        scrubber.stopMotor();
    }

}
