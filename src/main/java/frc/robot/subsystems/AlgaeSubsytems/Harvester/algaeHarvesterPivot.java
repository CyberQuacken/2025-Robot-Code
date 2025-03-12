package frc.robot.subsystems.AlgaeSubsytems.Harvester;

import com.google.flatbuffers.Constants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.algaeHarvesterConstants;
public class algaeHarvesterPivot extends SubsystemBase{
    
    private final SparkMax pivotMotor;
    RelativeEncoder relEncoder;//IDK how were dealing with encoders irl, so I will have both
    AbsoluteEncoder absEncoder;
    PIDController pidController;
    double[] positions;
    int currPosition;
    double Angle;
    public algaeHarvesterPivot(int m_intakeMotor){
        pivotMotor = new SparkMax(m_intakeMotor, MotorType.kBrushless);
        relEncoder = pivotMotor.getEncoder();
        absEncoder = pivotMotor.getAbsoluteEncoder();
        currPosition = 0;
        positions = algaeHarvesterConstants.positions;
        pidController = new PIDController( 
            algaeHarvesterConstants.kP,
            algaeHarvesterConstants.kI,
            algaeHarvesterConstants.kD
        );
    }

    
    //TODO: limit switches with encoders
    public void pivot(int direction){
        double pidOutput = pidController.calculate(absEncoder.getPosition(), positions[currPosition]);
        pivotMotor.set(pidOutput);
        
 
       /*  if(absEncoder.getPosition() >= algaeHarvesterConstants.minPivot && absEncoder.getPosition() <= algaeHarvesterConstants.maxPivot ){ 
        pivotMotor.set(direction * algaeHarvesterConstants.pivotSpeed);
        } else if (absEncoder.getPosition() < algaeHarvesterConstants.minPivot) { 
            pivotMotor.set(1 * algaeHarvesterConstants.pivotSpeed);
        } else { 
            pivotMotor.set(-1 * algaeHarvesterConstants.pivotSpeed);
        } */
    }
    public void RawPivot(double speed){ 
        pivotMotor.set(speed);
    }
    public void rawPivotDirection(int direction) { 
        pivotMotor.set(algaeHarvesterConstants.pivotSpeed * direction);
    }
    public void setCurrentPosition(int desiredPosition) {
        currPosition = desiredPosition;
      }
      public int getCurrentPosition(){
        return currPosition;
      }
    @Override
    public void periodic() { 
        Angle = absEncoder.getPosition();
    }
    public void stop() { 
        pivotMotor.stopMotor();
    }
}
