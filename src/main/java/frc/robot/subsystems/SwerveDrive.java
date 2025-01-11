package frc.robot.subsystems;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/*
 * This is the swerve drive class
 * for calculating where to drive
 * and sending directions to the SwerveModules
 */
public class SwerveDrive extends SubsystemBase{
    //Attributes
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;
    private final AHRS gyro = new AHRS(AHRS.NavXComType.kUSB2); // dependant on what gyroscope we use
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
        DriveConstants.kFrontLeftDrivingCanId,
        DriveConstants.kFrontLeftTurningCanId,
        DriveConstants.kFrontLeftChassisAngularOffset);
  
    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
        DriveConstants.kFrontRightDrivingCanId,
        DriveConstants.kFrontRightTurningCanId,
        DriveConstants.kFrontRightChassisAngularOffset);
  
    private final MAXSwerveModule m_backLeft = new MAXSwerveModule(
        DriveConstants.kBackLeftDrivingCanId,
        DriveConstants.kBackLeftTurningCanId,
        DriveConstants.kBackLeftChassisAngularOffset);
  
    private final MAXSwerveModule m_backRight = new MAXSwerveModule(
        DriveConstants.kBackRightDrivingCanId,
        DriveConstants.kBackRightTurningCanId,
        DriveConstants.kBackRightChassisAngularOffset);

    //Constructor
    public SwerveDrive()
    {
        //.swerveModules = new SwerveModule[4]; //Creates Swerve Modules

        kinematics = new SwerveDriveKinematics
        (
            new Translation2d(Units.inchesToMeters(12),Units.inchesToMeters(12)),
            new Translation2d(Units.inchesToMeters(12),Units.inchesToMeters(-12)),
            new Translation2d(Units.inchesToMeters(-12),Units.inchesToMeters(12)),
            new Translation2d(Units.inchesToMeters(-12),Units.inchesToMeters(-12))
        );// this values are not true to our robot

        /*
         * kinematics = new SwerveDriveKinematices (
         * new Translation2d (aka cord points)(units.conversion(Ydistance) , units.conversion(Xdistance))
         * new Translation2d (aka cord points)(units.conversion(-Ydistance) , units.conversion(Xdistance))
         * new Translation2d (aka cord points)(units.conversion(-Ydistance) , units.conversion(-Xdistance))
         * new Translation2d (aka cord points)(units.conversion(Ydistance) , units.conversion(-Xdistance))
         * );
         * + / - signs are not written with specfics and neither is the x or y accurate to cordnates
         */

        
        odometry = new SwerveDriveOdometry
        (
            kinematics,
            gyro.getRotation2d(), // gyro is currently using a false class, as such it will show error until we use the proper class
            new SwerveModulePosition[]{new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition()},
            new Pose2d(0,0,new Rotation2d())
        );
        /*
        * odometry = new SwerveDriveOdometry(
        * kinematics // i believe this assigns the odometry we are making to kinematics, considering kinematics is the one calculating
        * gyro.getAngle() // this is pretty useless as periodicly() the odometer will be updated on current gyro angle and swerveModule states
        * new swerveModulePosition[]{new position, new position, new position, new position} creates a new array for the odometer to use, and fills it will empty positions    
        * new Pose2d(0,0 ,  new Rotation2d()) // x= 0 , y = 0 , heading 0
        * // all new object are to fill the odometer with nonexistent values to be filled when updated
        * );
        */
    }

    
    // In robot container this is used every second or so
    // Take inputed values (from controller sticks), if drive will be relative to field, and if rate should be limited
    public void drive()
    {
        //test ChassisSpeeds
        ChassisSpeeds testSpeeds = new ChassisSpeeds(Units.inchesToMeters(14),Units.inchesToMeters(4),Units.degreesToRadians(30));
        //States for each module
        SwerveModuleState[] swerveModuleState = kinematics.toSwerveModuleStates(testSpeeds);
        
        m_frontLeft.setDesiredState(swerveModuleState[0]);//Front-Left
        m_frontRight.setDesiredState(swerveModuleState[1]);//Front-Right
        m_backLeft.setDesiredState(swerveModuleState[2]);//Back-Left
        m_backRight.setDesiredState(swerveModuleState[3]);//Back-Right
    }
    // Fetch the current module positions
    public SwerveModuleState[] getModuleStates(){ 
        return   new SwerveModuleState[] {
                  m_frontLeft.getState(),
                  m_frontRight.getState(),
                  m_backLeft.getState(),
                  m_backRight.getState()
              };
      }
    
    @Override
    public void periodic()
    {
        // Update the odometry
        odometry.update(gyro.getRotation2d(), new SwerveModulePosition[]{
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
             m_backRight.getPosition()
        });
    }
}
