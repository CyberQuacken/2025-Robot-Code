package frc.robot.commands.CoralClawCommandss;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralClawSubsystem;

public class manuoverCoral extends Command{
    CoralClawSubsystem coralClaw;
    
    public manuoverCoral (CoralClawSubsystem subsystem){
        coralClaw = subsystem;

        addRequirements(coralClaw);
    }

    @Override
    public void execute(){
        coralClaw.intakeMotor();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
     // might want to change it
    @Override
    public void end(boolean interrupted){
        coralClaw.stopMotor();
    }
}
