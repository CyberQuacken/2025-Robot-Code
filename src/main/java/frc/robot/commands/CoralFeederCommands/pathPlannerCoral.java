package frc.robot.commands.CoralFeederCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.coralFeederConstants;
import frc.robot.subsystems.CoralFeederSubsystem;

public class pathPlannerCoral extends Command{
    CoralFeederSubsystem coralFeeder;
    
    public pathPlannerCoral (CoralFeederSubsystem subsystem){
        coralFeeder = subsystem;

        addRequirements(coralFeeder);
    }

    @Override
    public void execute(){
        coralFeeder.testMotor(coralFeederConstants.clawIntakeSpeed);
        new WaitCommand(1);//TODO: Mess around with this number
        coralFeeder.stopMotor();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
     // might want to change it
    @Override
    public void end(boolean interrupted){
        coralFeeder.stopMotor();
    }
}
