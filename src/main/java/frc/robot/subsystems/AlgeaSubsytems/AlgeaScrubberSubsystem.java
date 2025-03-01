package frc.robot.subsystems.AlgeaSubsytems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgeaScrubberSubsystem extends SubsystemBase{
    
    private SparkMax motor;

    public void AlgeaScrubberSubsystem(int motorID){
        motor = new SparkMax(motorID, MotorType.kBrushless);
    }

    public void moveMotor(){
        motor.set(.25);
    }

    public void stopMotor(){
        motor.stopMotor();
    }
}
