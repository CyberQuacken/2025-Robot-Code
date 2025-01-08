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
