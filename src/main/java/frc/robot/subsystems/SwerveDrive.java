package frc.robot.subsystems;

/*
 * This is the swerve drive class
 * for calculating where to drive
 * and sending directions to the SwerveModules
 */
public class SwerveDrive extends SubsystemBase{
    //Attributes
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;
    Gyroscope gyro;
    SwerveModule[] swerveModules;

    //Constructor
    public SwerveDrive()
    {
        swerveModules = new SwerveModule[4]; //Creates Swerve Modules

        kinematics = newSwerveDriveKinematics
        (
            new Translation2d(Units.inchesToMeters(12.5),Units.inchesToMeters(12.5))
            new Translation2d(Units.inchesToMeters(12.5),Units.inchesToMeters(-12.5))
            new Translation2d(Units.inchesToMeters(-12.5),Units.inchesToMeters(12.5))
            new Translation2d(Units.inchesToMeters(-12.5),Units.inchesToMeters(-12.5))
        );

        /*
         * kinematics = new SwerveDriveKinematices (
         * new Translation2d (aka cord points)(units.conversion(Ydistance) , units.conversion(Xdistance))
         * new Translation2d (aka cord points)(units.conversion(-Ydistance) , units.conversion(Xdistance))
         * new Translation2d (aka cord points)(units.conversion(-Ydistance) , units.conversion(-Xdistance))
         * new Translation2d (aka cord points)(units.conversion(Ydistance) , units.conversion(-Xdistance))
         * );
         * + / - signs are not written with specfics and neither is the x or y accurate to cordnates
         */

         // gyro = new Gyro // setup gyro here, thats about it

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
    public void drive(){
        gyro = new Gyroscope();

        odometry = new SwerveDriveOdometry
        (
            kinematics
            gyro.getAngle()
            new SwerveModulePosition[]{new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition()}
            new Pose2d(0,0,new Rotation2d())
        );
    }
}
