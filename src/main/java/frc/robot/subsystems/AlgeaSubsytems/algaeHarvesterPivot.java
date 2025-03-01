package frc.robot.subsystems.AlgeaSubsytems;

import com.google.flatbuffers.Constants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.algaeHarvesterConstants;
public class algaeHarvesterPivot extends SubsystemBase{
    
    private final SparkMax pivotMotor;
    RelativeEncoder relEncoder;//IDK how were dealing with encoders irl, so I will have both
    AbsoluteEncoder absEncoder;

    public algaeHarvesterPivot(int m_intakeMotor){
        pivotMotor = new SparkMax(m_intakeMotor, MotorType.kBrushless);
        relEncoder = pivotMotor.getEncoder();
        absEncoder = pivotMotor.getAbsoluteEncoder();
    }


    //TODO: limit switches with encoders
    public void pivot(int direction){
        if(absEncoder.getPosition() >= algaeHarvesterConstants.minPivot && absEncoder.getPosition() <= algaeHarvesterConstants.maxPivot ){ 
        pivotMotor.set(direction * algaeHarvesterConstants.pivotSpeed);
        } else if (absEncoder.getPosition() < algaeHarvesterConstants.minPivot) { 
            pivotMotor.set(1 * algaeHarvesterConstants.pivotSpeed);
        } else { 
            pivotMotor.set(-1 * algaeHarvesterConstants.pivotSpeed);
        }
    }
    public void stop() { 
        pivotMotor.stopMotor();
    }
}
