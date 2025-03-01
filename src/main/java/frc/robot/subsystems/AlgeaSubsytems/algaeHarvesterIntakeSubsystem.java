package frc.robot.subsystems.AlgeaSubsytems;

import com.google.flatbuffers.Constants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.algaeHarvesterConstants;
public class algaeHarvesterIntakeSubsystem extends SubsystemBase{
    
    private final SparkMax intakeMotor;
    public algaeHarvesterIntakeSubsystem(int m_intakeMotor){
        intakeMotor = new SparkMax(m_intakeMotor, MotorType.kBrushless);
    }

    public void intake(){
        intakeMotor.set(algaeHarvesterConstants.algaeHarvesterIntakeSpeed);
    }
    public void stop() { 
        intakeMotor.stopMotor();
    }
}
