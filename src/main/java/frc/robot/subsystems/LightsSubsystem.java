package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightsSubsystem extends ExampleSubsystem
{

    final Spark lights;//channel?
    
    lightSubsytem(int lightsPort){
        lights = new Spark(0);
    }

    public void setColor(int pulseWidth){
        lights.set(pulseWidth);//conversions
    }
    public void turnLightsOff(){
        lights.set(0);
    }
    
}
