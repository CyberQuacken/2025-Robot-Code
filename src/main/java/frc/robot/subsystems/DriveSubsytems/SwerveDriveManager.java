package frc.robot.subsystems.DriveSubsytems;

import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.limelightAutoConstants;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Function;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PPLTVController;
import com.studica.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveManager extends SubsystemBase{
    public SwerveDrive driveSystem;

    private boolean autoDrive = false;
    private boolean align = true;

    private double desiredAngle;

    private boolean isBlue = false;

    // when true, align to the left of the april tag on the reef, when false go to the right
    private boolean alginOnLeft = false;

    private int stage = 0;
    
    public RobotConfig config;

    private PIDController distancePIDController;
    private PIDController rotateToHeadingPIDController;
    private PIDController horizontalPIDController;
    private PIDController coordinatePIDController;
    private double alignment = limelightAutoConstants.alignmentOffset; // starts left

    private final SwerveDrivePoseEstimator m_PoseEstimator;

    private String alignVal;

    /**
     * 
     * @param driveSystem
     * Subsytem that controls drive
     * @param aprilTaglist
     * team specific aprilTags
     */
    public SwerveDriveManager(int[] aprilTaglist){
        driveSystem = new SwerveDrive();
        driveSystem.gyro.resetDisplacement();

        m_PoseEstimator = new SwerveDrivePoseEstimator(
            driveSystem.kinematics,
            driveSystem.gyro.getRotation2d(),
            driveSystem.modulePosition,
            new Pose2d());

        distancePIDController = new PIDController(
            limelightAutoConstants.distance_kP,
            limelightAutoConstants.distance_kI,
            limelightAutoConstants.distance_kD);

        horizontalPIDController = new PIDController(
            limelightAutoConstants.horizontal_kP, 
            limelightAutoConstants.horizontal_kI, 
            limelightAutoConstants.horizontal_kD);

        coordinatePIDController = new PIDController(
        limelightAutoConstants.coordinate_kP,
        limelightAutoConstants.coordinate_kI,
        limelightAutoConstants.coordinate_kD);

        rotateToHeadingPIDController = new PIDController(
            limelightAutoConstants.rotation_kP,
            limelightAutoConstants.rotation_kI,
            limelightAutoConstants.rotation_kD);
        SmartDashboard.putString("State: " , "manual");
      

        SmartDashboard.putString("State: " , "manual");
        if(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("") != null){
        driveSystem.odometry.resetPose(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("").pose);
        }

        SmartDashboard.putNumber("Distance_P", limelightAutoConstants.distance_kP);
        SmartDashboard.putNumber("Distance_I", limelightAutoConstants.distance_kI);
        SmartDashboard.putNumber("Distance_D", limelightAutoConstants.distance_kD);

        SmartDashboard.putNumber("Horizontal_P", limelightAutoConstants.horizontal_kP);
        SmartDashboard.putNumber("Horizontal_I", limelightAutoConstants.horizontal_kI);
        SmartDashboard.putNumber("Horizontal_D", limelightAutoConstants.horizontal_kD);

        SmartDashboard.putNumber("coordinate_P", limelightAutoConstants.coordinate_kP);
        SmartDashboard.putNumber("coordinate_I", limelightAutoConstants.coordinate_kI);
        SmartDashboard.putNumber("coordinate_D", limelightAutoConstants.coordinate_kD);

        SmartDashboard.putNumber("Rotation_P", limelightAutoConstants.rotation_kP);
        SmartDashboard.putNumber("Rotation_I", limelightAutoConstants.rotation_kI);
        SmartDashboard.putNumber("Rotation_D", limelightAutoConstants.rotation_kD);
          
 //The autobuilder below doesnt have errors, but it will remain commented out for safety
/*  config = new RobotConfig(50, 1.89
 , new ModuleConfig(2, 5, alignment, null, null, stage)
 , new Translation2d[]{new Translation2d()}); */

 RobotConfig config = null;
 try{
   config = RobotConfig.fromGUISettings();
 } catch (Exception e) {
   // Handle exception as needed
   e.printStackTrace();
 }

 AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveSystem.driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5, 0, 1), // Translation PID constants
                    new PIDConstants(5, 0.0, 0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  /*   
  try { 
    /* 
    config = RobotConfig.fromGUISettings();
    AutoBuilder.configure(
            this::getPose,  //Pose Supplier
            this::resetOdometry, //Pose consumer
            this::getSpeeds,   //Speeds supplier
            (speeds, feedforwards) -> driveSystem.driveRobotRelative(speeds), //Output
            new PPHolonomicDriveController(
                new PIDConstants(500, 0, 20),
                new PIDConstants(75, 0, 10)
        ), // Controller
            config, // Robot Configuration
            () -> { 
                var alliance = DriverStation.getAlliance();
                if(alliance.isPresent()) { 
                    return alliance.get() == DriverStation.Alliance.Red;
                }    
      return false;  //Field side, I think blue is false(not sure though) This will have to be dealt with before any actual games
    }
    
     //Set requirements
  );
  
 } catch(Exception e) { 
    AutoBuilder.configureCustom(null, null, null, alginOnLeft);
    e.printStackTrace();
 } 
 */

    
    }


    public Pose2d getPoseEstimation(){
        return driveSystem.getPoseEstimation();
    }




    public void ManageSwerveSystem(double controllerXvalue, double controllerYvalue, double controllerRotValue, boolean fieldCentric, boolean slewRate, boolean reducedSpeed){
        SmartDashboard.putNumber("Distance", controllerRotValue);
        SmartDashboard.putNumber("PosX", driveSystem.relativeOdometry.getPoseMeters().getX());
        SmartDashboard.putBoolean("ROBOT MODE", fieldCentric); // field centric will be green, robot centric wils be red
        //SmartDashboard.putNumber("distance", driveSystem.gyro.getDisplacementX());
         if(reducedSpeed){
            controllerXvalue /= 4;
            controllerYvalue /= 4;
            controllerRotValue /= 4;
        }
        if (!autoDrive){
            SmartDashboard.putString("State: " , "manual");
            driveSystem.drive(controllerXvalue, controllerYvalue, controllerRotValue, fieldCentric, slewRate);
        }
        else{ // if auto
            SmartDashboard.putString("State: " , "auto");
            /* removed to test on the spot pathfinding .-.
            double otherHorizontalDistance = LimelightHelpers.getTargetPose_CameraSpace("")[2]*Math.tan(LimelightHelpers.getTX("") * (Math.PI/180));

            if(align){
                if(LimelightHelpers.getTV("") == true){ 
                    
                driveSystem.drive(
                    moveToDistance(LimelightHelpers.getTargetPose_CameraSpace("")[2], 1.5),
                    //orbitOnAngle(-LimelightHelpers.getTX(""),SmartDashboard.getNumber("Degrees off", 10)),//orbitOnAngle(LimelightHelpers.getTargetPose_CameraSpace("")[4], 0),
                    orbitOnAngle(-otherHorizontalDistance, .25),
                    rotateToHeading(-LimelightHelpers.getTargetPose_CameraSpace("")[4], 0),
                    false, true);
                    
                    alignOnTag(alignment);// .25 // is at 13 and -13
                    // if at position (say) at position
                    SmartDashboard.putBoolean("Aligned", alignedOnTag());
                }
            }
            else{
                moveDistance(.6, 0);
            }
            */
        }
    }

    /**
     * 
     * @param distance
     * distance in meters relative to robot
     * 
     * @return
     * speed calculated input for controller
     */
    public double moveToDistance(double currentDistance, double desiredDistance){ // if a single value doesnt work, just make it zero
        SmartDashboard.putNumber("currentDistance: ", currentDistance);
        SmartDashboard.putNumber("desiredDistance", desiredDistance);
        double calculatedSpeed = distancePIDController.calculate(-currentDistance, -desiredDistance);
        SmartDashboard.putNumber("calculated Speed", calculatedSpeed);
        return calculatedSpeed;
    }

    public double moveToCoordinateDistance(double currentDistance, double desiredDistance){
        double calculatedSpeed = coordinatePIDController.calculate(-currentDistance, -desiredDistance);
        return calculatedSpeed;
    }

    public double rotateToHeading(double Currentangle, double desiredAngle){ // rotate robot to specific heading
        double calculatedSpeed = rotateToHeadingPIDController.calculate(-Currentangle, -desiredAngle);
        return calculatedSpeed;
    }

    public double orbitOnAngle(double currentDegreesOff, double desiredDegreesOff){ // move robot around point, such as april tag
        double calculatedSpeed = horizontalPIDController.calculate(-currentDegreesOff, desiredDegreesOff);
        return calculatedSpeed;
    }

    public double centerLimelight(){
        double calculatedSpeed = LimelightHelpers.getTX("") * -limelightAutoConstants.alignment_kP * Math.PI;
        return calculatedSpeed;
    }

    public void OperateReef(){
        // if (aligning){}
        // when at bottom, if at position, aligning == false
        if (stage == 1){
            // FirstStage go to april tag
                moveToTag();
                // if at proper position, change stage to 2
                if(atAprilTag()){
                    stage ++;
                }
        }
        else if (stage == 2){
            // second stage, align on april tag
            alignOnTag();

            // if at proper position, change stage to 3
            if(alignedOnTag()){
                stage ++;
            }
        }
        else if (stage == 3){
            // third stage, prepare odomenty
            resetRelativeOdometry();
            stage ++;
        }
        else if (stage == 4){
            // move forward .6 meters
            moveDistance(.6, 0);

            // if at position, stage = 0
        }
    }

    @Override
    public void periodic()
    {

        driveSystem.updateSystem();
        updateRobotPos();

        /* 
        distancePIDController.setP(SmartDashboard.getNumber("Distance_P", 0.0));
        distancePIDController.setI(SmartDashboard.getNumber("Distance_I", 0.0));
        distancePIDController.setD(SmartDashboard.getNumber("Distance_D", 0.0));

        horizontalPIDController.setP(SmartDashboard.getNumber("Horizontal_P", 0.0));
        horizontalPIDController.setI(SmartDashboard.getNumber("Horizontal_I", 0.0));
        horizontalPIDController.setD(SmartDashboard.getNumber("Horizontal_D", 0.0));

        coordinatePIDController.setP(SmartDashboard.getNumber("coordinate_P", 0.0));
        coordinatePIDController.setI(SmartDashboard.getNumber("coordinate_I", 0.0));
        coordinatePIDController.setD(SmartDashboard.getNumber("coordinate_D", 0.0));

        rotateToHeadingPIDController.setP(SmartDashboard.getNumber("Rotation_P", 0.0));
        rotateToHeadingPIDController.setI(SmartDashboard.getNumber("Rotation_I", 0.0));
        rotateToHeadingPIDController.setD(SmartDashboard.getNumber("Rotation_D", 0.0));
        */

        SmartDashboard.putNumber("X pose", driveSystem.getPoseEstimation().getX());
        SmartDashboard.putNumber("Y pose", driveSystem.getPoseEstimation().getY());
        SmartDashboard.putNumber("Desired Angle", desiredAngle);

    }

    public AHRS getGyro(){
        return driveSystem.gyro;
    }

    // might be removed
    public void updateRobotPos(){
        m_PoseEstimator.update(
        driveSystem.gyro.getRotation2d(), 
        driveSystem.modulePosition);

        if (LimelightHelpers.getTV("")){
            Pose2d visionMeasurement2d = LimelightHelpers.getBotPose2d("");

            m_PoseEstimator.addVisionMeasurement(visionMeasurement2d, LimelightHelpers.getLatency_Capture(""));
        }

        
        SmartDashboard.putNumber("X : ", m_PoseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Y : ", m_PoseEstimator.getEstimatedPosition().getY());
    }

    public void updateAlliance(boolean isBlue){
        this.isBlue = isBlue;
    }


    //--------------------
    public void toggleLimelightAuto(){
        autoDrive = !autoDrive;
        stage = 1;
    }

    public void setLimelightAuto(boolean value){
        autoDrive = value;
    }

    public void toggleAlignment(){
        align = !align;
        
        if(!align) { 
            alignVal = "Translation";
        } else { 
            alignVal = "Rotation";
        }
        resetRelativeOdometry();
        SmartDashboard.putString("Auto motion", alignVal);
    }

    public void moveDistance(double xDistance, double yDistance){
        driveSystem.drive(
        -moveToCoordinateDistance(driveSystem.relativeOdometry.getPoseMeters().getX(), -xDistance),
        -moveToCoordinateDistance(driveSystem.relativeOdometry.getPoseMeters().getY(), -yDistance),
          0, false, true);
    }

    public void moveToTag(double desiredDistanceOff, double desiredDegreesOff){
        driveSystem.drive(
            moveToDistance(LimelightHelpers.getTargetPose_CameraSpace("")[2], desiredDistanceOff), 
            orbitOnAngle(LimelightHelpers.getTargetPose_CameraSpace("")[4], desiredDegreesOff),
            centerLimelight(), 
            false, true);
    }

    public void moveToTag(){
        driveSystem.drive(
            moveToDistance(LimelightHelpers.getTargetPose_CameraSpace("")[2], 1.5), 
            orbitOnAngle(LimelightHelpers.getTargetPose_CameraSpace("")[4], 0),
            centerLimelight(), 
            false, true);
    }

    public void alignOnTag(){
        driveSystem.drive(
            moveToDistance(LimelightHelpers.getTargetPose_CameraSpace("")[2], 1.5),
            //orbitOnAngle(-LimelightHelpers.getTX(""),SmartDashboard.getNumber("Degrees off", 10)),//orbitOnAngle(LimelightHelpers.getTargetPose_CameraSpace("")[4], 0),
            orbitOnAngle(-LimelightHelpers.getTargetPose_CameraSpace("")[2]*Math.tan(LimelightHelpers.getTX("") * (Math.PI/180)), -.25),
            rotateToHeading(-LimelightHelpers.getTargetPose_CameraSpace("")[4], 0),
            false, true);
            desiredAngle = driveSystem.gyro.getAngle() + -LimelightHelpers.getTargetPose_CameraSpace("")[4];
    }

    public void alignOnTag(double distance){
        driveSystem.drive(
            moveToDistance(LimelightHelpers.getTargetPose_CameraSpace("")[2], 1),
            //orbitOnAngle(-LimelightHelpers.getTX(""),SmartDashboard.getNumber("Degrees off", 10)),//orbitOnAngle(LimelightHelpers.getTargetPose_CameraSpace("")[4], 0),
            //orbitOnAngle(-LimelightHelpers.getTargetPose_CameraSpace("")[2]*Math.tan(LimelightHelpers.getTX("") * (Math.PI/180)), distance),
            orbitOnAngle(-LimelightHelpers.getTX(""), distance),
            rotateToHeading(-LimelightHelpers.getTargetPose_CameraSpace("")[4], 0),
            false, true);
    }

    public void resetRelativeOdometry(){
        driveSystem.relativeOdometry.resetPose(new Pose2d(0,0, new Rotation2d()));
        driveSystem.drive(.1, 0 ,0 , false, true);
    }

    public double getDesiredAngle(){
        return desiredAngle;
    }

    public boolean atAprilTag(){
        //returns true if robot is at april tag
        return false;
    }

    public boolean alignedOnTag(){
        // returns truenif robot is aligned correctly at the april tag
        return horizontalPIDController.atSetpoint() && distancePIDController.atSetpoint() && rotateToHeadingPIDController.atSetpoint();
    }

    public boolean distanceMoved(){
        // returns true if robot has move desired distance
        return false;
    }

    public void alignLeft(){
        //alignment = limelightAutoConstants.alignmentOffset;
        alginOnLeft = true;
    }

    public void alignRight(){
        //alignment = -limelightAutoConstants.alignmentOffset;
        alginOnLeft = false;
    }

    public void alignCenter(){
        alignment = 0;
    }


public Pose2d getPose(){
    return driveSystem.getPose();
}
public ChassisSpeeds getSpeeds(){
    return driveSystem.getSpeeds();
}
public void resetOdometry(Pose2d pose) {
    driveSystem.resetOdometry(pose);
}
}
