package frc.robot.subsystems.DriveSubsytems;

import org.dyn4j.exception.SameObjectException;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.ModulePosition;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PPLTVController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants;
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
    private double prevSpeed = 0.0;
    private double speed = 0.0;
    private double prevRot = 0.0;
    private final double deadzone = 0.05;
    public double fieldCentricHeading;

    private SwerveDrivePoseEstimator poseEstimator;
    //Structs for AdvantageScope Simulation
    boolean limelightAuto = false;
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
    SwerveDriveOdometry relativeOdometry;
    AHRS gyro;
    private double m_currentTranslationMag = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentRotation = 0.0;
    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-7;

  
    private final MAXSwerveModule m_backLeft = new MAXSwerveModule(
        DriveConstants.kBackLeftDrivingCanId,
        DriveConstants.kBackLeftTurningCanId,
        DriveConstants.kBackLeftChassisAngularOffset);
        
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
        DriveConstants.kFrontLeftDrivingCanId,
        DriveConstants.kFrontLeftTurningCanId,
        DriveConstants.kFrontLeftChassisAngularOffset);
  
    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
        DriveConstants.kFrontRightDrivingCanId,
        DriveConstants.kFrontRightTurningCanId,
        DriveConstants.kFrontRightChassisAngularOffset);
  
    private final MAXSwerveModule m_backRight = new MAXSwerveModule(
        DriveConstants.kBackRightDrivingCanId,
        DriveConstants.kBackRightTurningCanId,
        DriveConstants.kBackRightChassisAngularOffset);
    double prevTime = WPIUtilJNI.now();

    double time = WPIUtilJNI.now();

    public SwerveModulePosition[] modulePosition = new SwerveModulePosition[]{
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
    };

    //Constructor
    public SwerveDrive()
    {
        for(int i = 0; i < 4; i++){
            currStates[i] = new SwerveModuleState();
        }

        gyro = new AHRS(NavXComType.kMXP_SPI);

        kinematics = new SwerveDriveKinematics
        (
            new Translation2d(Units.inchesToMeters(12),Units.inchesToMeters(12)),
            new Translation2d(Units.inchesToMeters(12),Units.inchesToMeters(-12)),
            new Translation2d(Units.inchesToMeters(-12),Units.inchesToMeters(12)),
            new Translation2d(Units.inchesToMeters(-12),Units.inchesToMeters(-12))
        );// this values are not true to our robot

        relativeOdometry = new SwerveDriveOdometry(
        kinematics, 
        new Rotation2d(0),
        new SwerveModulePosition[]{new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition()},
        new Pose2d(0,0,new Rotation2d()));
        
        odometry = new SwerveDriveOdometry
        (
            kinematics,
            gyro.getRotation2d(), 
            new SwerveModulePosition[]{new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition()},
            new Pose2d(0,0,new Rotation2d(0))
        );


        //AUTO
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e){
            e.printStackTrace();
        }
        
       
  
    poseEstimator = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(gyro.getAngle()), modulePosition, pose);//I put the rotation2d to fix an error, the param was previously null
    resetOdometry(new Pose2d(0,0,new Rotation2d(180)));
    zeroHeading();
}
//Auto methods
public Pose2d getPose(){
    return odometry.getPoseMeters();
}
public ChassisSpeeds getSpeeds(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
}
/* */
public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] { 
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        }, pose
    );
}

public void resetOdometry() {
    odometry.resetPosition(
        Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] { 
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        }, new Pose2d(0,0, new Rotation2d(180))
    );
}

public void driveRobotRelative(ChassisSpeeds RobotRelativeSpeeds){
    ChassisSpeeds fixedSpeeds = RobotRelativeSpeeds.times(-1);
    fixedSpeeds.omegaRadiansPerSecond = fixedSpeeds.omegaRadiansPerSecond*-1;
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
    ChassisSpeeds.fromFieldRelativeSpeeds(fixedSpeeds,getPose().getRotation())
    );
    
    
        SmartDashboard.putNumber("FLA", swerveModuleStates[0].angle.getRadians());
        SmartDashboard.putNumber("FRA", swerveModuleStates[1].angle.getRadians());
        SmartDashboard.putNumber("BLA", swerveModuleStates[2].angle.getRadians());
        SmartDashboard.putNumber("BRA", swerveModuleStates[3].angle.getRadians());

        SmartDashboard.putNumber("FLD", swerveModuleStates[0].speedMetersPerSecond);
        SmartDashboard.putNumber("FRD", swerveModuleStates[1].speedMetersPerSecond);
        SmartDashboard.putNumber("BLD", swerveModuleStates[2].speedMetersPerSecond);
        SmartDashboard.putNumber("BRD", swerveModuleStates[3].speedMetersPerSecond);

    //setModuleStates(swerveModuleStates);\
    
    /* 
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
    */
    
    //m_frontLeft.setDesiredRot(swerveModuleStates[0].angle);
    //m_frontRight.setDesiredRot(swerveModuleStates[1].angle);
    //m_backLeft.setDesiredRot(swerveModuleStates[2].angle);
    //m_backRight.setDesiredRot(swerveModuleStates[3].angle);

    /* 
    swerveModuleStates[3].angle = swerveModuleStates[3].angle;
    swerveModuleStates[2].angle = swerveModuleStates[2].angle;
    swerveModuleStates[1].angle = swerveModuleStates[1].angle;
    swerveModuleStates[0].angle = swerveModuleStates[0].angle;

    swerveModuleStates[3].speedMetersPerSecond = - swerveModuleStates[3].speedMetersPerSecond;
    swerveModuleStates[2].speedMetersPerSecond = - swerveModuleStates[2].speedMetersPerSecond;
    swerveModuleStates[1].speedMetersPerSecond = - swerveModuleStates[1].speedMetersPerSecond;
    swerveModuleStates[0].speedMetersPerSecond = - swerveModuleStates[0].speedMetersPerSecond;
*/

    setModuleStates(swerveModuleStates);

    //swervePublisher.set(swerveModuleStates);
    //setDesiredStates(swerveModuleStates);
}
public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  } 

    public void toggleLimelightAuto(){
        limelightAuto = !limelightAuto;
    }

    // In robot container this is used every second or so
    // Take inputed values (from controller sticks), if drive will be relative to field, and if rate should be limited
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit)
    {   
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
        if (rateLimit){

            //Vector: direction and magnitude
            double inputTranslationDir = Math.atan2(ySpeed, xSpeed);//Find angle given two points on circle
            double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));//Pythagorean theorem

            double directionSlewRate;
            if (m_currentTranslationMag != 0.0){ //If driving
                directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
            } else { 
                directionSlewRate = 500.0;
            }
      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);//Change between goal and current in radians
      //System.out.println(angleDif);
      if (angleDif < 0.45*Math.PI) {
        //System.out.println("SMALL!");
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        //System.out.println("BIG");
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        //System.out.println("MEDIUM!");
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommand = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommand = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


        }else { 
            xSpeedCommand = xSpeed;
            ySpeedCommand = ySpeed;
            m_currentRotation = rot;
        }
        
        

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

        SmartDashboard.putNumber("FLA", swerveModuleStates[0].angle.getRadians());
        SmartDashboard.putNumber("FRA", swerveModuleStates[1].angle.getRadians());
        SmartDashboard.putNumber("BLA", swerveModuleStates[2].angle.getRadians());
        SmartDashboard.putNumber("BRA", swerveModuleStates[3].angle.getRadians());

        SmartDashboard.putNumber("FLD", swerveModuleStates[0].speedMetersPerSecond);
        SmartDashboard.putNumber("FRD", swerveModuleStates[1].speedMetersPerSecond);
        SmartDashboard.putNumber("BLD", swerveModuleStates[2].speedMetersPerSecond);
        SmartDashboard.putNumber("BRD", swerveModuleStates[3].speedMetersPerSecond);

        
        
        m_frontLeft.setDesiredState(swerveModuleStates[0]);//Front-Left
        m_frontRight.setDesiredState(swerveModuleStates[1]);//Front-Right
        m_backLeft.setDesiredState(swerveModuleStates[2]);//Back-Left
        m_backRight.setDesiredState(swerveModuleStates[3]);//Back-Right
    

        //setDesiredStates(swerveModuleStates);
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

    public void updateSystem()
    {
        pose = getPose();
        publisher.set(pose);
        swervePublisher.set(currStates); // BRINg BACK
        // Update the odometry
        modulePosition = new SwerveModulePosition[]{
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
             m_backRight.getPosition()
        };
        //System.out.println(gyro.getRotation2d());
        relativeOdometry.update(new Rotation2d(0), modulePosition);
        System.out.println(m_backLeft.getPosition().distanceMeters/Constants.ModuleConstants.kWheelCircumferenceMeters);
        odometry.update(gyro.getRotation2d(), modulePosition);
        double x = getSpeeds().vxMetersPerSecond;
        double y = getSpeeds().vyMetersPerSecond;
        prevSpeed = speed;
        prevTime = time;
        time = WPIUtilJNI.now();
        speed = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
        SmartDashboard.putNumber("Speed", speed);

        double acceleration = (speed - prevSpeed)/(time-prevTime);
        SmartDashboard.putNumber("Acceleration", acceleration);

        poseEstimator.update(gyro.getRotation2d(), modulePosition);
        if(LimelightHelpers.getTV("")){
            poseEstimator.addVisionMeasurement(LimelightHelpers.getBotPose2d(""), LimelightHelpers.getLatency_Pipeline(""));
        }
        SmartDashboard.putNumber("gyro: ", gyro.getAngle());
        SmartDashboard.putNumber("Esitimated X", poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Esitimated Y", poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Odometer", getPose().getRotation().getDegrees());
    }

    public Pose2d getPoseEstimation(){
        return poseEstimator.getEstimatedPosition();
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

    public void StoreFieldCentricHeading (){
        fieldCentricHeading = gyro.getAngle();
    }
    public void SetGyroToFieldCentricHeading (){
        gyro.reset();
        gyro.setAngleAdjustment(fieldCentricHeading);
    }

  
    }
