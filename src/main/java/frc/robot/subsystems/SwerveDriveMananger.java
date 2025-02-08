package frc.robot.subsystems;

import frc.robot.LimelightHelpers;
import frc.robot.Constants.limelightAutoConstants;
import frc.robot.Objects.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDriveMananger {
    private SwerveDrive driveSystem;
    private int[] currentAprilTags;

    private boolean autoDrive = false;

    //current april tag robot is going to
    private int desiredAprilTagIndex;                          

    //next april tag after robot is finished with desiredAprilTag
    private int queudAprilTagIndex;

    private PIDController distancePIDController;
    private PIDController rotateToHeadingPIDController;
    private PIDController horizontalPIDController;
    
    public void SwerveDriveMananger(SwerveDrive driveSystem, int[] aprilTaglist){
        this.driveSystem = driveSystem;
        currentAprilTags = aprilTaglist;

        distancePIDController = new PIDController(
            limelightAutoConstants.distance_kP,
            limelightAutoConstants.distance_kI,
            limelightAutoConstants.distance_kD);

        SmartDashboard.putString("State: " , "manual");
    }
    public void SwerveDriveMananger(SwerveDrive driveSystem, int[] aprilTaglist, boolean startInAuto){
        this.driveSystem = driveSystem;
        currentAprilTags = aprilTaglist;
        autoDrive = startInAuto;
        if (autoDrive){
            SmartDashboard.putString("State: " , "auto");
        }
        else{
            SmartDashboard.putString("State: " , "manual");
        }
    }

    public void ManangSwerveSystem(double controllerXvalue, double controllerYvalue, double controllerRotValue, boolean fieldCentric, boolean slewRate){
        if (autoDrive){
            SmartDashboard.putString("State: " , "manual");
            driveSystem.drive(controllerXvalue, controllerYvalue, controllerRotValue, fieldCentric, slewRate);
        }
        else{ // if auto
            SmartDashboard.putString("State: " , "auto");
            driveSystem.drive(LimelightHelpers.getTargetPose_CameraSpace("")[2],0,0,false, true);
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
    public double moveToDistance(double distance){ // if a single value doesnt work, just make it zero
        double calculatedSpeed = distancePIDController.calculate(distance);
        return calculatedSpeed;
    }

    public void rotateToHeading(double angle){ // rotate robot to specific heading

        //return calculateSpeed
    }

    public void orbitOnAngle(double degreesOff){ // move robot around point, such as april tag

        //return calculateSpeed
    }

    public void goTo(int aprilTagIndex){

        if (canBeSpotted(aprilTagIndex)){ // april tag can be spooted (reef and proccesor)
            if(true){ // april tag spooted
                OperateTags(aprilTagIndex);
            }
            else{
                pathfind(true, aprilTagIndex);
            }
        }
        else{ // april tags can not be spooted (feeder and barge)
            if(true){ // if at suitable Position to move to tag
                OperateTags(aprilTagIndex);
            }
            else{ // pathfind to suitable Position
                pathfind(true, aprilTagIndex);
            }
        }

    }
    public void goTo(Vector desiredPosition){

    }

    public void OperateTags(int aprilTagIndex){
        SmartDashboard.putString("auto Operation", "Operating: " + currentAprilTags[aprilTagIndex]);
    }

    public void pathfind(boolean rotated, int tag){

    }

    public void pathfind(double angle, int tag){

    }

    public void pathfind(boolean rotated, Vector position){

    }

    public void pathfind(double angle, Vector position){

    }

    public boolean canBeSpotted(int aprilTagIndex){
        if (aprilTagIndex > 0 && aprilTagIndex <= 8){
            return true;
        }
        else {
            return false;
        }
    }
}
