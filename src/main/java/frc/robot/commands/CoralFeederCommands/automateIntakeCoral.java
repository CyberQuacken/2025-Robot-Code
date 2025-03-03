package frc.robot.commands.CoralFeederCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralFeederSubsystem;

public class automateIntakeCoral extends Command{
    CoralFeederSubsystem coralFeeder;
    boolean hasCoral;
    
    public automateIntakeCoral (CoralFeederSubsystem subsystem){
        coralFeeder = subsystem;
        hasCoral = coralFeeder.testForCoral();

        addRequirements(coralFeeder);
    }

    @Override
    public void execute(){
        coralFeeder.intakeMotor();
    }

    @Override
    public boolean isFinished(){
        // if nothing has change, continue the program
        if(hasCoral == coralFeeder.testForCoral()){
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
        coralFeeder.stopMotor();
    }

}