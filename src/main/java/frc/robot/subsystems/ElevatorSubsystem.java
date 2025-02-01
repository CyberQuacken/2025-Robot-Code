// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.elevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  SparkMax leftMotor;
  SparkMax rightMotor;
  int[] positions; // list of all positions
  int currentPosition; // index of the position we want the elevator to be att
  
  // possible to make indivual PIDocntrller or motor specific PIDcontroller
  PIDController pidController;

  RelativeEncoder leftEncoder;
  RelativeEncoder rightEncoder;
  // encoder for the motors

  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem(int leftMotorID, int rightMotorID) {
    leftMotor = new SparkMax(leftMotorID, MotorType.kBrushless);
    rightMotor = new SparkMax(rightMotorID, MotorType.kBrushless);

    positions = elevatorConstants.positions; // copies saved positions from constant
    currentPosition = 0; // or whatever the number is from the starting position

    // <I do not know if we need encoders for both motors>
    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    //not sure what this dos completely..
    //leftPIDController = leftMotor.getClosedLoopController();
    //rightPIDController = rightMotor.getClosedLoopController();

    pidController = new PIDController(
      elevatorConstants.kP,
      elevatorConstants.kI,
      elevatorConstants.kD);
  }

/**
 * move motor will be the standard command for the robot
 * every second it will use the PID to move to its desired position
 * to change where the Elevator moves to, use the setCurrentPosition method
 */
  public void moveMotors(){
    // the PID controller requestest a distance, im not sure how to get distance or what it correlates to
    leftMotor.set(pidController.calculate(leftEncoder.getVelocity(), positions[currentPosition]));
    rightMotor.set(pidController.calculate(rightEncoder.getVelocity(), positions[currentPosition]));
    //sends data to motors
  }

  public void setCurrentPosition(int desiredPosition) {
    currentPosition = desiredPosition;
  }
  public int getCurrentPosition(){
    return currentPosition;
  }
}
