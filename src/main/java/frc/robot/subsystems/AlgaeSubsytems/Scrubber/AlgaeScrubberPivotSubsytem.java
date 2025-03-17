package frc.robot.subsystems.AlgaeSubsytems.Scrubber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.algaeScrubberConstants;
import frc.robot.Constants.coralFeederConstants;

public class AlgaeScrubberPivotSubsytem extends SubsystemBase{

    private SparkMax motor;
    private SparkBaseConfig motorConfig;
    private RelativeEncoder relativeEncoder;

    private PIDController pidController = new PIDController(
        algaeScrubberConstants.kP, 
        algaeScrubberConstants.kI, 
        algaeScrubberConstants.kD);

    
    public AlgaeScrubberPivotSubsytem (int motorID){
        motor = new SparkMax(motorID, MotorType.kBrushless);

        relativeEncoder = motor.getEncoder();
        //relativeEncoder.setPosition(0);
    }


    public void moveMotorOut(){
       double pidOutput = pidController.calculate(relativeEncoder.getPosition(), algaeScrubberConstants.scrubberOutPosition);
       //motor.set(-Math.max(-.2,pidOutput));
       motor.set(Math.min(.2,pidOutput));
    }

    public void moveMotorIn(){
        double pidOutput = pidController.calculate(relativeEncoder.getPosition(), algaeScrubberConstants.scrubberInPosition);
        motor.set(Math.max(-.2,pidOutput));
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

    public void testMotor(double speed){
        motor.set(speed);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("scrubber pivot" , relativeEncoder.getPosition());
    }
}
