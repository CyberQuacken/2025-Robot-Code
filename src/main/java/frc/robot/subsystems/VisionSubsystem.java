package frc.robot.subsystems;

import org.opencv.video.Video;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

public class VisionSubsystem extends SubsystemBase {
    @Override
    public void periodic() { 
        
        double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
        RawFiducial[] data = LimelightHelpers.getRawFiducials("");
        if(tx != 0.0) { 
           //System.out.println("AprilTag detected!");
            //System.out.println(data[0].id);
        }
        
    }
    public boolean getDetection() { 
        if (LimelightHelpers.getTX("") != 0.0) { 
            return true;
        } else { 
            return false;
        }
    }
    public double getOffset() { 
        return LimelightHelpers.getTX("");
    }
    
    
    public void run() { 
        double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
        double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
        double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
        boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?

        double txnc = LimelightHelpers.getTXNC("");  // Horizontal offset from principal pixel/point to target in degrees
        double tync = LimelightHelpers.getTYNC("");  // Vertical  offset from principal pixel/point to target in degrees
        
        LimelightHelpers.setPipelineIndex("", 0);
        // Set a custom crop window for improved performance (-1 to 1 for each value)
        LimelightHelpers.setCropWindow("", -0.5, 0.5, -0.5, 0.5);


        // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
        LimelightHelpers.setCameraPose_RobotSpace("", 
            0.5,    // Forward offset (meters)
            0.0,    // Side offset (meters)
            0.5,    // Height offset (meters)
            0.0,    // Roll (degrees)
            30.0,   // Pitch (degrees)
            0.0     // Yaw (degrees)
        );

        // Set AprilTag offset tracking point (meters)
        LimelightHelpers.setFiducial3DOffset("", 
            0.0,    // Forward offset
            0.0,    // Side offset  
            0.5     // Height offset
        );

        // Configure AprilTag detection
        LimelightHelpers.SetFiducialIDFiltersOverride("", new int[]{1, 2, 3, 4}); // Only track these tag IDs
        LimelightHelpers.SetFiducialDownscalingOverride("", 2.0f); // Process at half resolution for improved framerate and reduced range
        //System.out.println(LimelightHelpers.getFiducialID(""));
        RawFiducial[] data = LimelightHelpers.getRawFiducials("");
        //data[data.length-1].id
        //System.out.println("testing");
        //System.out.println(LimelightHelpers.getTX(""));

    }
}
