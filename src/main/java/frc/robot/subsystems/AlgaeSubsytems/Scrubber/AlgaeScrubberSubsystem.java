package frc.robot.subsystems.AlgaeSubsytems.Scrubber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.algaeScrubberConstants;

public class AlgaeScrubberSubsystem extends SubsystemBase{
    
    private SparkMax motor;
    private CommandXboxController controller;

    public AlgaeScrubberSubsystem(int motorID, CommandXboxController inpuController){
        motor = new SparkMax(motorID, MotorType.kBrushless);
        controller = inpuController;
    }

    public void scrub(){
        motor.set(algaeScrubberConstants.algaeScrubberSpeed);
               SmartDashboard.putString("Scrubber", "moving");
    }

    public void moveMotor(){
        motor.set(.25);
    }

    public void stopMotor(){
        motor.stopMotor();
    }

    public void override (){
        motor.set(-MathUtil.applyDeadband(controller.getRightY(), OIConstants.kDeadband));
        SmartDashboard.putString("Scrubber", "overriden");
    }

    public void testMotor(double speed){
        motor.set(speed);
    }
}
