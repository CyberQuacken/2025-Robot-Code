package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralClawSubsystem extends SubsystemBase {

    SparkMax motor;
    // distance sensor, used to see if we have a coral
    boolean hasCoral;
    
    public CoralClawSubsystem (int motorID){
        motor = new SparkMax(motorID, MotorType.kBrushless);
    }


    public void stopMotor(){
        motor.stopMotor();
    }

    public void intakeMotor(){
        motor.set(.5);
    }

    public void outakeMotor(){
        motor.set(-.5);
    }

    public boolean testForCoral(){
        // if distance ims too small, return true
        return false;
    }

    @Override
    public void periodic() {
        hasCoral = testForCoral();
    }
}
