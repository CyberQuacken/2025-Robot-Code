// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.elevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  SparkMax leftMotor;
  SparkMax rightMotor;
  int[] positions; // list of all positions
  int currentPosition; // index of the position we want the elevator to be att
  //PID leftMotorPID;
  //PID rightMotorPID
  AbsoluteEncoder encoder;
  // encoder for the motors

  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem(int leftMotorID, int rightMotorID) {
    leftMotor = new SparkMax(leftMotorID, MotorType.kBrushless);
    rightMotor = new SparkMax(rightMotorID, MotorType.kBrushless);

    positions = elevatorConstants.elevatorPosition; // copies saved positions from constant
    currentPosition = 0; // or whatever the number is from the statering position

    // <I do not know if we need encoders for both motors>
    encoder = leftMotor.getAbsoluteEncoder();

    //set up MOTORs

    //Configure Motor PIDS
  }

/**
 * move motor will be the standard command for the robot
 * every second it will use the PID to move to its desired position
 * to change where the Elevator moves to, use the setCurrentPosition method
 */
  public void moveMotors(){
    //PID calculates

    //sends data to motors
  }

  public void setCurrentPosition(int desiredPosition) {
    currentPosition = desiredPosition;
  }
  public int getCurrentPosition(){
    return currentPosition;
  }
}
