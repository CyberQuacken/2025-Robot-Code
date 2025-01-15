package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralClawPivotSubsystem extends SubsystemBase{

    SparkMax pivotMotor;

    PIDController pidController;

    int[] positions;
    int currentPosition;

    // <The plan is to have move be the default command, and we just change where it goes>
    public CoralClawPivotSubsystem (int pivotMotorID) {
        pivotMotor = new SparkMax(pivotMotorID, MotorType.kBrushless);

        //positions = ClawConstants.positions
        currentPosition = 0;

        //create PIDController
    }

    public void move() {
        //do pid calculations via current position
        pivotMotor.set(0);
    }

    // all index positions not actual positions
    public void setCurrentPosition (int desiredPosition){
        currentPosition = desiredPosition;
    }

    public int getCurrentPosition () {
        return currentPosition;
    }
}
