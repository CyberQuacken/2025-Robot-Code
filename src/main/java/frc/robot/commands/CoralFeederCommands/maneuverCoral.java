package frc.robot.commands.CoralFeederCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.coralFeederConstants;
import frc.robot.subsystems.CoralFeederSubsystem;

public class maneuverCoral extends Command{
    CoralFeederSubsystem coralFeeder;
    
    public maneuverCoral (CoralFeederSubsystem subsystem){
        coralFeeder = subsystem;

        addRequirements(coralFeeder);
    }

    @Override
    public void execute(){
        coralFeeder.intakeMotor(coralFeederConstants.clawIntakeSpeed);
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
