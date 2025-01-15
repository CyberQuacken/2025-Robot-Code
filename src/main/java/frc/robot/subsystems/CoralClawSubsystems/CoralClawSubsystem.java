// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralClawSubsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralClawSubsystem extends SubsystemBase {
  SparkMax upperMotor;
  SparkMax lowerMotor;

  SparkMaxConfig upperMotorConfig;
  SparkMaxConfig lowerMotorConfig;

  public CoralClawSubsystem(int upperMotorID, int lowerMotorID) {
    //create and assign motor ports
    lowerMotor = new SparkMax(lowerMotorID, MotorType.kBrushless);
    upperMotor = new SparkMax(upperMotorID, MotorType.kBrushless);

    // create configurations for motors
    upperMotorConfig = new SparkMaxConfig();
    lowerMotorConfig = new SparkMaxConfig();
    // set motor config, upper is inverted and lower is not. all motors brake when not moving
    upperMotorConfig.inverted(true).idleMode(IdleMode.kBrake);
    lowerMotorConfig.inverted(false).idleMode(IdleMode.kBrake);

    // assign motors config
    upperMotor.configure(upperMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    lowerMotor.configure(lowerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void stopMotors () {
    upperMotor.stopMotor();
    lowerMotor.stopMotor();
  }

  /**
   * intake and outtake both take the absolute value of whatever value is given 
   * and sets the motor to that speed 
   * @param intakeSpeed
   * the values are absolute to cause less confusion with use and not worry about breaking
   */
  public void intake (double intakeSpeed) {
    Math.abs(intakeSpeed);
    upperMotor.set(intakeSpeed);
    lowerMotor.set(intakeSpeed);
  }

  public void outtake (double outtakeSpeed) {
    Math.abs(outtakeSpeed);
    upperMotor.set(-outtakeSpeed);
    lowerMotor.set(-outtakeSpeed);
  }
}
