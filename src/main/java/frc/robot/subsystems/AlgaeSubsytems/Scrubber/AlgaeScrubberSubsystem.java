package frc.robot.subsystems.AlgaeSubsytems.Scrubber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.algaeScrubberConstants;

public class AlgaeScrubberSubsystem extends SubsystemBase{
    
    private SparkMax motor;

    public AlgaeScrubberSubsystem(int motorID){
        motor = new SparkMax(motorID, MotorType.kBrushless);
    }

    public void scrub(){
        motor.set(algaeScrubberConstants.algaeScrubberSpeed);
    }

    public void moveMotor(){
        motor.set(.25);
    }

    public void stopMotor(){
        motor.stopMotor();
    }

    public void testMotor(double speed){
        motor.set(speed);
    }
}
