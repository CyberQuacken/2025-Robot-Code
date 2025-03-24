package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralFeederSubsystem extends SubsystemBase {

    SparkMax motor;
    DigitalInput sensor;
    // distance sensor, used to see if we have a coral
    boolean hasCoral;
    
    public CoralFeederSubsystem (int motorID, int sensorPort){
        motor = new SparkMax(motorID, MotorType.kBrushless);
        sensor = new DigitalInput(sensorPort);
    }


    public void stopMotor(){
        motor.stopMotor();
    }

    public void intakeMotor(double speed){
        motor.set(-speed);
    }

    public void testMotor(double speed){
        motor.set(-speed);
    }

    public boolean testForCoral(){
        // if distance ims too small, return true
        SmartDashboard.putBoolean("distance sensed", sensor.get());
        return sensor.get();
        //return false;//TODO: Add algorithm for this once we have system
    }

    @Override
    public void periodic() {
        hasCoral = testForCoral();
        SmartDashboard.putBoolean("CORAL : ", hasCoral);
    }
}
