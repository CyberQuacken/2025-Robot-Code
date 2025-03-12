package frc.robot.subsystems.AlgaeSubsytems.Harvester;

import com.google.flatbuffers.Constants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.algaeHarvesterConstants;
public class algaeHarvesterIntakeSubsystem extends SubsystemBase{
    
    private final SparkMax intakeMotor;
    private boolean intake;
    public algaeHarvesterIntakeSubsystem(int m_intakeMotor){
        intakeMotor = new SparkMax(m_intakeMotor, MotorType.kBrushless);
        intake = true;
    }

    public void intake(){
        intakeMotor.set(algaeHarvesterConstants.algaeHarvesterIntakeSpeed);
    }
    public void stop() { 
        intakeMotor.stopMotor();
    }

    public boolean getIntake(){
        return intake;
    }

    public void setIntake(boolean value){
        intake = value;
    }
}
