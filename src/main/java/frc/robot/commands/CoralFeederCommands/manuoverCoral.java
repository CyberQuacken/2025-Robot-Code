package frc.robot.commands.CoralFeederCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralFeederSubsystem;

public class manuoverCoral extends Command{
    CoralFeederSubsystem coralFeeder;
    
    public manuoverCoral (CoralFeederSubsystem subsystem){
        coralFeeder = subsystem;

        addRequirements(coralFeeder);
    }

    @Override
    public void execute(){
        coralFeeder.intakeMotor();
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
