package frc.robot.subsystems;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.SwerveUtils;

/*
 * This is the swerve drive class
 * for calculating where to drive
 * and sending directions to the SwerveModules
 */
public class SwerveDrive extends SubsystemBase
{
    private Rotation2d[] tempRots = new Rotation2d[4];
    private double prevXSpeed = 0.0;
    private double prevYSpeed = 0.0;
    private double prevRot = 0.0;
    private final double deadzone = 0.02;
    //Structs for AdvantageScope Simulation
    Pose2d pose = new Pose2d();
    StructPublisher<Pose2d> publisher = 
    NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();
    StructArrayPublisher<SwerveModuleState> swervePublisher =
    NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
    
    //Sim states
    SwerveModuleState[] currStates = {new SwerveModuleState(), new SwerveModuleState(),new SwerveModuleState(),new SwerveModuleState()};



    //Attributes
    private Rotation2d[] rots = new Rotation2d[4];
    RobotConfig config;
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;
    private AHRS gyro;
    private double m_currentTranslationMag = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentRotation = 0.0;
    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-7;


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
        for(int i = 0; i < 4; i++){
            currStates[i] = new SwerveModuleState();
        }
        //.swerveModules = new SwerveModule[4]; //Creates Swerve Modules
        gyro = new AHRS(NavXComType.kUSB1);
        //swerveModules = new SwerveModule[4]; //Creates Swerve Modules

        kinematics = new SwerveDriveKinematics
        (
            new Translation2d(Units.inchesToMeters(12),Units.inchesToMeters(12)),
            new Translation2d(Units.inchesToMeters(12),Units.inchesToMeters(-12)),
            new Translation2d(Units.inchesToMeters(-12),Units.inchesToMeters(12)),
            new Translation2d(Units.inchesToMeters(-12),Units.inchesToMeters(-12))
        );// this values are not true to our robot


        
        odometry = new SwerveDriveOdometry
        (
            kinematics,
            gyro.getRotation2d(), 
            new SwerveModulePosition[]{new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition()},
            new Pose2d(0,0,new Rotation2d())
        );


        //AUTO
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e){
            e.printStackTrace();
            //System.out.println("AUTO!!!!");
        }

        AutoBuilder.configure(
            this::getPose,
            this::resetOdometry,
            this::getSpeeds,
            (speeds, feedforwards) -> driveRobotRelative(speeds),
            new PPLTVController(0.02),
            config,
            () -> {     
      return false;
    },
    this //Set requirements
  );       
    }
//Auto methods
private Pose2d getPose(){
    return odometry.getPoseMeters();
}
public ChassisSpeeds getSpeeds(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
}
public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        Rotation2d.fromDegrees(-gyro.getAngle()),
        new SwerveModulePosition[] { 
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        }, pose
    );
}
public void driveRobotRelative(ChassisSpeeds RobotRelativeSpeeds){
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
    ChassisSpeeds.fromFieldRelativeSpeeds(RobotRelativeSpeeds,getPose().getRotation())
    );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(swerveModuleStates);
}
public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }


    // In robot container this is used every second or so
    // Take inputed values (from controller sticks), if drive will be relative to field, and if rate should be limited
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative)
    {
        double past = xSpeed;
        //If change in speed is greater than (deadzone), set change to deadzone. 
        //This is the purpose of the giant if statements. 
        if(prevXSpeed > 0) { 
            if((xSpeed - prevXSpeed) > 0 && (xSpeed - prevXSpeed) > deadzone) {//Increasing
                xSpeed = prevXSpeed + deadzone;
            } else if ((xSpeed - prevXSpeed) < 0 && (xSpeed - prevXSpeed) < -deadzone) { 
                xSpeed = prevXSpeed - deadzone;
            }
        } else if (prevXSpeed < 0) { 
            if((prevXSpeed - xSpeed) > 0 && (prevXSpeed - xSpeed) > deadzone) {//Increasing
                xSpeed = prevXSpeed - deadzone;
            } else if ((prevXSpeed - xSpeed) < 0 && (prevXSpeed - xSpeed) < -deadzone) { 
                xSpeed = prevXSpeed + deadzone;
            }
        }


        if(prevYSpeed > 0) { 
            if((ySpeed - prevYSpeed) > 0 && (ySpeed - prevYSpeed) > deadzone) {//Increasing
                ySpeed = prevYSpeed + deadzone;
            } else if ((ySpeed - prevYSpeed) < 0 && (ySpeed - prevYSpeed) < -deadzone) { 
                ySpeed = prevYSpeed - deadzone;
            }
        } else if (prevYSpeed < 0) { 
            if((prevYSpeed - ySpeed) > 0 && (prevYSpeed - ySpeed) > deadzone) {//Increasing
                ySpeed = prevYSpeed - deadzone;
            } else if ((prevYSpeed - ySpeed) < 0 && (prevYSpeed - ySpeed) < -deadzone) { 
                ySpeed = prevYSpeed + deadzone;
            }
       
        //xSpeed = Axis 1, YSpeed = Axis 0
        }
        prevXSpeed = xSpeed;
        prevYSpeed = ySpeed;
        //System.out.println("Past: " + past + " Present: " + xSpeed); // Making sure limiter works
        double xSpeedCommand;
        double ySpeedCommand;
     
        xSpeedCommand = xSpeed;
        ySpeedCommand = ySpeed;
        m_currentRotation = rot;
        
        

        //Convert input to m/s chassis speeds.
        double xSpeedDelivered = xSpeedCommand * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeedCommand * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed; 

        //Convert chassis speeds to usable module states
        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(-gyro.getAngle()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));        
        //System.out.println(swerveModuleStates[0].angle);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

        
        
        m_frontLeft.setDesiredState(swerveModuleStates[0]);//Front-Left
        m_frontRight.setDesiredState(swerveModuleStates[1]);//Front-Right
        m_backLeft.setDesiredState(swerveModuleStates[2]);//Back-Left
        m_backRight.setDesiredState(swerveModuleStates[3]);//Back-Right
    

        setDesiredStates(swerveModuleStates);
        SmartDashboard.putNumber("TargetPos", LimelightHelpers.getCameraPose_TargetSpace("")[5]);
        SmartDashboard.putNumber("RobotSpace", LimelightHelpers.getTargetPose_RobotSpace("")[5]);
        SmartDashboard.putNumber("ID", LimelightHelpers.getTargetCount(""));
        SmartDashboard.putNumber("rot", rotDelivered);
        SmartDashboard.putNumber("jostick", rot);

    }
    public void moveRot(double rot, boolean fieldRelative) { 
        double rotDelivered = Math.toRadians(rot) * DriveConstants.kMaxAngularSpeed;
        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(fieldRelative ? 
        ChassisSpeeds.fromFieldRelativeSpeeds(0,0,rotDelivered, Rotation2d.fromDegrees(-gyro.getAngle())) : 
        new ChassisSpeeds(0,0, rotDelivered));
        m_frontLeft.setDesiredState(swerveModuleStates[0]);//Front-Left
        m_frontRight.setDesiredState(swerveModuleStates[1]);//Front-Right
        m_backLeft.setDesiredState(swerveModuleStates[2]);//Back-Left
        m_backRight.setDesiredState(swerveModuleStates[3]);//Back-Right
        
    }
    /* 
    public void moveToHeading(double x, double y, boolean fieldRelative){
        double currentHeadingDegrees = gyro.getAngle();
        double headingDiffrence = currentHeadingDegrees - LimelightHelpers.getCameraPose_TargetSpace("")[5];
        double offsetHeadingDegrees = MathUtil.inputModulus(headingDiffrence, -180, 180);

        double pidRotation =
            ca
    }
     */

    public Rotation2d[] getLastRots() { 
        return rots;
    }
    public Rotation2d[] getCurrRots(){ 
        return new Rotation2d[]  { 
            m_frontLeft.getState().angle,
            m_frontRight.getState().angle,
            m_backLeft.getState().angle,
            m_backRight.getState().angle,
        };
    }


    public void turn(Rotation2d[] rots) { 
        m_frontLeft.setDesiredRot(rots[0]);
        m_frontRight.setDesiredRot(rots[1]);
        m_backLeft.setDesiredRot(rots[2]);
        m_backRight.setDesiredRot(rots[3]);
    }

        public SwerveModuleState[] getModuleStates(){ 
        return   new SwerveModuleState[] {
                  m_frontLeft.getState(),
                  m_frontRight.getState(),
                  m_backLeft.getState(),
                  m_backRight.getState()
              };
      }
    private void setDesiredStates(SwerveModuleState[] states){
        currStates = states;
    }

    @Override
    public void periodic()
    {
        pose = getPose();
        publisher.set(pose);
        swervePublisher.set(currStates);
        // Update the odometry
        odometry.update(gyro.getRotation2d(), new SwerveModulePosition[]{
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
             m_backRight.getPosition()
        });
       //FL, FR, BL, BR
        double loggingState[] = { 
            currStates[0].angle.getDegrees(),
            currStates[0].speedMetersPerSecond,
            currStates[1].angle.getDegrees(),
            currStates[1].speedMetersPerSecond,            
            currStates[2].angle.getDegrees(),
            currStates[2].speedMetersPerSecond,            
            currStates[3].angle.getDegrees(),
            currStates[3].speedMetersPerSecond,            
        };
        double x = getSpeeds().vxMetersPerSecond;
        double y = getSpeeds().vyMetersPerSecond;
        double speed = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
        SmartDashboard.putNumber("Speed", speed);
        SmartDashboard.putNumberArray("SwerveModuleStates", loggingState);
        


    }
    public void setX() {
         m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45))); 
      }
      
    public void zeroHeading(){
        gyro.reset();
    }

  
    }
