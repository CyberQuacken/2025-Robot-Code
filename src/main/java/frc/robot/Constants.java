// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 1;
    public static final int kScorerControllerPort = 0;

  }
  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final boolean fieldRelative = true;
    public static final double kDeadband = 0.15;
  }
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 3.0; // percent per second (1 = 100%)
    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs,  we will likely have to change these
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kBackLeftDrivingCanId = 1;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kBackRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 6;
    public static final int kBackLeftTurningCanId = 5;
    public static final int kFrontRightTurningCanId = 7;
    public static final int kBackRightTurningCanId = 8;

    public static final int kGryoi2cPort = 1;

    public static final boolean kGyroReversed = false;

    //PID for auto horizontal alignment
    public static final double limelight_auto_kp_horizontal = 0.0054;
    public static final double limelight_auto_ki_horizontal = 0.0044;
    public static final double limelight_auto_kd_horizontal = 0.0;

    //PID fpr auto vertical/distance alignment
    public static final double limelight_auto_kp_vertical = 0.095;
    public static final double limelight_auto_ki_vertical = 0.0021;
    public static final double limelight_auto_kd_vertical = 0.0;

    //PID for auto alignment in facing april tags
    public static final double limelight_auto_kp_rotation = 0.003;
  }
    
public static class elevatorConstants {

  //depends on motor data?
  public static final double[] positions ={
    0, // intake
    15, // level one // doesnt work
    20.5, // level two
    38.3, // level three
    69, // level four
  };
  
  public static final int intakePositionIndex = 0;

  public static final int levelOnePositionIndex = 1;
  public static final int levelTwoPositionIndex = 2;
  public static final int levelThreePositionIndex = 3;
  public static final int levelFourPositionIndex = 4;

  // temp values for PID
  public static final double kP = 0.12;
  public static final double kI = 0.0015;
  public static final double kD = 0.0001; 

  public static final int leftMotorCanID = 9;
  public static final int rightMotorCanID = 10;//Double check these
}

public static class algeaScrubberConstants {
  public static final int algeaScrubberMotorID = 12;
  public static final int algeaScrubberPivotMotorID = 13;
}

public static class coralFeederConstants {

  // preset values for how much the claw takes in <needs to be changed before use>
  public static final double clawIntakeSpeed = .5;
  public static final int motorID = 11;
}

public static class lightConstants {
  public static final int upperLightsPort = 1;
  public static final int lowerLightsPort = 1;


  // list of all premade patterns 
  public static final LEDPattern[] upperLightsPatterns = {
    LEDPattern.solid(Color.kBlack), // off
    null
  };

  public static final LEDPattern[] lowerlightsPattern = {
    LEDPattern.solid(Color.kBlack), // off
    null
  };
}

public static class limelightAutoConstants{
  // all tags for red team
  private final int[] redAprilTags = new int[]{
    1, // processer
    2, 2, 2, 2, 2, 2, // reef, starts at point facing barge, and rotates clockwise
    2, 2, // feeder tags
    2, 2, 2, 2 // barge tags (probally wont be used)
  };
  
  // all tags for blue team
  private final int[] blueAprilTags = new int[]{
    1, // processer
    2, 2, 2, 2, 2, 2, // reef, starts at point facing barge, and rotates clockwise
    2, 2, // feeder tags
    2, 2, 2, 2 // barge tags (probally wont be used)
  };


  public static final double distance_kP = .150;
  public static final double distance_kI = .0001;
  public static final double distance_kD = 0.0;

  public static final double horizontal_kP = .006;
  public static final double horizontal_kI = .0005;
  public static final double horizontal_kD = 0.0;

  public static final double coordinate_kP = .3;
  public static final double coordinate_kI = 0.1;
  public static final double coordinate_kD = 0.0;

  public static final double rotation_kP = .003;
  public static final double rotation_kI = .00;
  public static final double rotation_kD = .00;

  public static final double alignment_kP = .004;


  public static final double alignmentOffset = 1; //how far robot needs to move to align on reef pvc // + is left, - is right
  public static final double distanceOffset = 1; // how far robot is supposed to be (verically) from limelight
}

public static class algaeHarvesterConstants { 
    public static final int algaeHarvesterIntakeSpeed = 1;
    public static final int pivotSpeed = 1;
    public static final int pivotMotorCANID = 13;
    public static final int intakeMotorCANID = 12;
    public static final double minPivot = 10.0; //TODO: Change placeholder, careful with this, requires irl data and can break robot
    public static final double maxPivot = 90.0; //TODO: change placeholder

    public static final double kP = 0.12;
    public static final double kI = 0.0015;
    public static final double kD = 0.0001; 


    //TODO: get irl data, we'll need more precise positions, zero encoder
    public static final double[] positions ={
      10, //Unextended
      45, // Medium
      80, //parallel-ish to the horizontal. 
    };
    public static final int verticalIndex = 0;
    public static final int mediumIndex = 1;
    public static final int horizontalIndex = 2;

}
}
