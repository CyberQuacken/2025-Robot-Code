package frc.robot.subsystems;

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
    private PIDController yawPIDController;
    
    public void SwerveDriveMananger(SwerveDrive driveSystem, int[] aprilTaglist){
        this.driveSystem = driveSystem;
        currentAprilTags = aprilTaglist;
        SmartDashboard.putString("State: " , "manual");
    }
    public void SwerveDriveMananger(SwerveDrive driveSystem, int[] aprilTaglist, boolean startInAuto){
        this.driveSystem = driveSystem;
        currentAprilTags = aprilTaglist;
        autoDrive = startInAuto;

        SmartDashboard.putString("State: " , "auto");
    }

    public void ManangSwerveSystem(double controllerXvalue, double controllerYvalue, double controllerRotValue, boolean fieldCentric, boolean slewRate){
        if (autoDrive){
            SmartDashboard.putString("State: " , "manual");
            driveSystem.drive(controllerXvalue, controllerYvalue, controllerRotValue, fieldCentric, slewRate);
        }
        else{ // if auto
            SmartDashboard.putString("State: " , "auto");
            goTo(desiredAprilTagIndex);
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

    /**
     * 
     * @param relativeX
     * distance in meters of new position relative to robot in x
     * ex: xr = 5, xp = 2, relativeX = -3
     */
    public void moveToX(double relativeX){

        //return calculateSpeed
    }

    /**
     * 
     * @param relativeY
     * distance in meters of new Position relative to robot in y
     */
    public void moveToY(double relativeY){


        //return calculateSpeed
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
