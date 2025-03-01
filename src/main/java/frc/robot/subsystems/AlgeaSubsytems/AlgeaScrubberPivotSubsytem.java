package frc.robot.subsystems.AlgeaSubsytems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgeaScrubberPivotSubsytem extends SubsystemBase{

    private SparkMax motor;
    private SparkBaseConfig motorConfig;
    private RelativeEncoder relativeEncoder;
    
    private boolean extended;
    
    
    public AlgeaScrubberPivotSubsytem (int motorID){
        motor = new SparkMax(motorID, MotorType.kBrushless);

        relativeEncoder = motor.getEncoder();
    }

    // temp, any values should be moved to constants soon
    public void moveMotorOut(){
        motor.set(.25);
    }

    public void moveMotorIn(){
        motor.set(-.25);
    }

    public void stopMotor(){
        motor.stopMotor();
    }

    public double getEncoderValue(){
        return relativeEncoder.getPosition();
    }

    public void resetEncoder(){
        relativeEncoder.setPosition(0);
    }
}
