// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.elevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  SparkFlex leftMotor;
  SparkFlex rightMotor;
  double[] positions; // list of all positions
  int currentPosition; // index of the position we want the elevator to be att
  DigitalInput theSwitch;

  // possible to make indivual PIDocntrller or motor specific PIDcontroller
  PIDController pidController;

  RelativeEncoder leftEncoder;
  RelativeEncoder rightEncoder;

  double averagePosition = 0;
  // encoder for the motors

  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem(int leftMotorID, int rightMotorID) {
    SmartDashboard.putNumber("move", currentPosition);
    leftMotor = new SparkFlex(leftMotorID, MotorType.kBrushless);
    rightMotor = new SparkFlex(rightMotorID, MotorType.kBrushless);

    theSwitch = new DigitalInput(1);

    positions = elevatorConstants.positions; // copies saved positions from constant
    currentPosition = 0; // or whatever the number is from the starting position

    // <I do not know if we need encoders for both motors>
    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();


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
    //double moveToPosition = SmartDashboard.getNumber("move", currentPosition); // temp variable to create ladder positions
    double pidOutput = pidController.calculate(averagePosition, positions[currentPosition]);
    boolean isNegative = pidOutput / Math.abs(pidOutput) == -1;
    //double pidOutput = pidController.calculate(averagePosition, moveToPosition); 
    // the PID controller requestest a distance, im not sure how to get distance or what it correlates to
    if (isNegative){
      leftMotor.set(-Math.max(-.2,pidOutput));//positions[currentPosition]));
      rightMotor.set(Math.max(-.2,pidOutput));//positions[currentPosition]));
      SmartDashboard.putString("Elevators", "Down");
    }
    else{
      leftMotor.set(-Math.min(.2,pidOutput));//positions[currentPosition]));
      rightMotor.set(Math.min(.2,pidOutput));//positions[currentPosition]));
      SmartDashboard.putString("Elevators", "Up");
    }
    SmartDashboard.putNumber("current Output", pidOutput);
    //sends data to motors
    //System.out.println(moveToPosition);
    //if (pidController.atSetpoint()){
    //  leftEncoder.setPosition(positions[currentPosition]);
    //  rightEncoder.setPosition(positions[currentPosition]);
    //}
  }

  public void setCurrentPosition(int desiredPosition) {
    currentPosition = desiredPosition;
  }
  public int getCurrentPosition(){
    return currentPosition;
  }
// name pending, its manual controls
  public void moveElevator(double speed){
    leftMotor.set(-speed);
    rightMotor.set(speed);
  }

  public double getAveragePosition(){
    return averagePosition;
  }

  @Override
  public void periodic(){
    //System.out.println("active");
    SmartDashboard.putNumber("leftMotor", -leftEncoder.getPosition());
    SmartDashboard.putNumber("RightMotor", rightEncoder.getPosition());
    averagePosition = (rightEncoder.getPosition() + (-leftEncoder.getPosition())) / (2.0);
    SmartDashboard.putNumber("average Encoder Position", averagePosition);
    //currentPosition = SmartDashboard.getNumber("moving position", 0);

    SmartDashboard.putNumber("desired Position Index", currentPosition);
    SmartDashboard.putNumber("desired Position", positions[currentPosition]);

    pidController.setP(SmartDashboard.getNumber("elevator P", elevatorConstants.kP));
    pidController.setI(SmartDashboard.getNumber("elevator I", elevatorConstants.kI));
    pidController.setD(SmartDashboard.getNumber("elevator D", elevatorConstants.kD));
    SmartDashboard.putBoolean("elevator all down", theSwitch.get());
  }

  public boolean getSwitch (){
    return theSwitch.get();
  }

  public void resetEncoder (){
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }
}
