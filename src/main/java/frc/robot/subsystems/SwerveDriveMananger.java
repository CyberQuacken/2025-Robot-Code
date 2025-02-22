package frc.robot.subsystems;

import frc.robot.LimelightHelpers;
import frc.robot.Constants.limelightAutoConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Objects.Vector;

import static edu.wpi.first.units.Units.Rotation;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.reduxrobotics.canand.CanandAddress;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveMananger extends SubsystemBase{
    public SwerveDrive driveSystem;
    private int[] currentAprilTags;

    private boolean autoDrive = false;
    private boolean align = true;

    private boolean isBlue = false;
    private boolean useLimelight = false;

    // when true, align to the left of the april tag on the reef, when false go to the right
    private boolean alginOnLeft = false;

    //current april tag robot is going to
    private int desiredAprilTagIndex;                          

    //next april tag after robot is finished with desiredAprilTag
    private int queudAprilTagIndex;

    private Vector currentPos = new Vector();
    private Vector desiredPose = new Vector(7.4, 4.7);
    
    public RobotConfig config;

    private PIDController distancePIDController;
    private PIDController rotateToHeadingPIDController;
    private PIDController horizontalPIDController;
    private PIDController coordinatePIDController;

    private final SwerveDrivePoseEstimator m_PoseEstimator;

    /**
     * 
     * @param driveSystem
     * Subsytem that controls drive
     * @param aprilTaglist
     * team specific aprilTags
     */
    public SwerveDriveMananger(int[] aprilTaglist){
        driveSystem = new SwerveDrive();
        currentAprilTags = aprilTaglist;
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
        if(LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("") != null){
        driveSystem.odometry.resetPose(LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("").pose);
        }

        SmartDashboard.putString("State: " , "manual");
        driveSystem.odometry.resetPose(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("").pose);


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
    }
    public SwerveDriveMananger(int[] aprilTaglist, boolean startInAuto){
        driveSystem = new SwerveDrive();
        currentAprilTags = aprilTaglist;
        autoDrive = startInAuto;

        m_PoseEstimator = new SwerveDrivePoseEstimator(
            driveSystem.kinematics,
            driveSystem.gyro.getRotation2d(),
            driveSystem.modulePosition,
            new Pose2d());


        if (autoDrive){
            SmartDashboard.putString("State: " , "auto");
        }
        else{
            SmartDashboard.putString("State: " , "manual");
        }
    }

    public void ManangSwerveSystem(double controllerXvalue, double controllerYvalue, double controllerRotValue, boolean fieldCentric, boolean slewRate){
        SmartDashboard.putNumber("Distance", controllerRotValue);
        SmartDashboard.putNumber("PosX", driveSystem.relativeOdometry.getPoseMeters().getX());
        //SmartDashboard.putNumber("distance", driveSystem.gyro.getDisplacementX());
        if (!autoDrive){
            SmartDashboard.putString("State: " , "manual");
            driveSystem.drive(controllerXvalue, controllerYvalue, controllerRotValue, fieldCentric, slewRate);
        }
        else{ // if auto
            double distanceX = desiredPose.X() - currentPos.X();
            double distanceY = desiredPose.Y() - currentPos.Y();
            SmartDashboard.putString("State: " , "auto");
            if(align){
                driveSystem.drive(
                    moveToDistance(LimelightHelpers.getTargetPose_CameraSpace("")[2], 1.5),
                    orbitOnAngle(-LimelightHelpers.getTX(""),SmartDashboard.getNumber("Degrees off", 13)),//orbitOnAngle(LimelightHelpers.getTargetPose_CameraSpace("")[4], 0),
                    rotateToHeading(-LimelightHelpers.getTargetPose_CameraSpace("")[4], 0),
                    false, true);
            }
            else{
                driveSystem.drive(moveToCoordinateDistance(-driveSystem.relativeOdometry.getPoseMeters().getX(), -.75),0,0,false,true);
            }
            /*
            moveToCoordinateDistance(-distanceX,0),
            moveToCoordinateDistance(-distanceY,0),
            controllerRotValue,
            true, true);  */


            //goTo(desiredAprilTagIndex);
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

    public void goTo(int aprilTagIndex){

        if (canBeSpotted(aprilTagIndex)){ // april tag can be spooted (reef and proccesor)
            if(true){ // april tag spooted
                OperateTags(aprilTagIndex);
            }
            else{
                //pathfind(true, aprilTagIndex);
            }
        }
        else{ // april tags can not be spooted (feeder and barge)
            if(true){ // if at suitable Position to move to tag
                OperateTags(aprilTagIndex);
            }
            else{ // pathfind to suitable Position
                //pathfind(true, aprilTagIndex);
            }
        }

    }
    public void goTo(Vector desiredPosition){

    }

    public void OperateTags(int aprilTagIndex){
        SmartDashboard.putString("auto Operation", "Operating: " + currentAprilTags[aprilTagIndex]);
    }

    public void OperateReef(){
        
    }

    public boolean canBeSpotted(int aprilTagIndex){
        if (aprilTagIndex > 0 && aprilTagIndex <= 8){
            return true;
        }
        else {
            return false;
        }
    }

    @Override
    public void periodic()
    {
        driveSystem.updateSystem();
        updateRobotPos();

        
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

    }

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
    }

    public void toggleAlignment(){
        align = !align;
    }
}
