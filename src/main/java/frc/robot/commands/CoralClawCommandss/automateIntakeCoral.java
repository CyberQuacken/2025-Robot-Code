package frc.robot.commands.CoralClawCommandss;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralClawSubsystem;

public class automateIntakeCoral extends Command{
    CoralClawSubsystem coralClaw;
    boolean hasCoral;
    
    public automateIntakeCoral (CoralClawSubsystem subsystem){
        coralClaw = subsystem;
        hasCoral = coralClaw.testForCoral();

        addRequirements(coralClaw);
    }

    @Override
    public void execute(){
        coralClaw.intakeMotor();
    }

    @Override
    public boolean isFinished(){
        // if nothing has change, continue the program
        if(hasCoral == coralClaw.testForCoral()){
            return false;
        }
        // else prepare to end the program
        else{
            //____I Don't think we need two cases, but just in case____
            // if claw started with coral
            if(hasCoral){
                return true;
            }
            // if claw is taking in coral
            if(!hasCoral){
                return true;
            }
        }
        return false;
    }
    
    @Override
    public void end(boolean interrupted){
        coralClaw.stopMotor();
    }

}