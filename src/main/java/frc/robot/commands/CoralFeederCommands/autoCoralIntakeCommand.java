package frc.robot.commands.CoralFeederCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.coralFeederConstants;
import frc.robot.subsystems.CoralFeederSubsystem;

public class autoCoralIntakeCommand extends Command{
    CoralFeederSubsystem coralFeeder;
    boolean hasCoral;
    
    public autoCoralIntakeCommand (CoralFeederSubsystem subsystem){
        coralFeeder = subsystem;
        hasCoral = coralFeeder.testForCoral();

        addRequirements(coralFeeder);
    }

    @Override
    public void execute(){
        coralFeeder.intakeMotor(coralFeederConstants.clawIntakeSpeed);
    }

    @Override
    public boolean isFinished(){
        // if nothing has change, continue the program
        if(coralFeeder.testForCoral()){
            return true;
        }
        else{
            return false;
        }
    }
    
    @Override
    public void end(boolean interrupted){
        coralFeeder.stopMotor();
    }

}