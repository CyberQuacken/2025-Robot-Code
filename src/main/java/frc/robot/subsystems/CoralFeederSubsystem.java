package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralFeederSubsystem extends SubsystemBase {

    SparkMax motor;
    // distance sensor, used to see if we have a coral
    boolean hasCoral;
    
    public CoralFeederSubsystem (int motorID){
        motor = new SparkMax(motorID, MotorType.kBrushless);
    }


    public void stopMotor(){
        motor.stopMotor();
    }

    public void intakeMotor(){
        motor.set(.4);
    }

    public void outakeMotor(){
        motor.set(-.5);
    }

    public void testMotor(double speed){
        motor.set(-speed);
    }

    public boolean testForCoral(){
        // if distance ims too small, return true
        return false;//TODO: Add algorithm for this once we have system
    }

    @Override
    public void periodic() {
        hasCoral = testForCoral();
    }
}
