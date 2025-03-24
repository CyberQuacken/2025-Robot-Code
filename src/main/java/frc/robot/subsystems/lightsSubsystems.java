package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class lightsSubsystems extends SubsystemBase{

    private AddressableLED lights;
    private AddressableLEDBuffer lightBuffer;

    /*
     * solid colors and their Spark Max value
     * 
     * 0.61-red
     * 0.65-orange
     * 0.69-yellow
     * 0.77-green
     * 0.83-sky blue
     * 0.85-dark blue
     * 0.91-violet
     * 0.57-hot pink
     * 0.93-white
     * 
     */

    private double[] patterns = {-.57, 0.65, 0.69, 0.77, 0.83, 0.85, 0.91, 0.57, 0.93};
    
    private int LEDindex;

    private Spark light;

    public lightsSubsystems(int port, int length) {
        //lights = new AddressableLED(0);
        //lights.setLength(length);
        //patterns = setpatterns;
        light = new Spark(1);
        lightBuffer = new AddressableLEDBuffer(length);
        LEDindex = 0;
        //setColor();
        on();

    }

    public void on() {
        //lights.start();
        // I believe that black just means not on for the lights
        setDefaultCommand(runPattern(patterns[LEDindex]).withName("Off"));
    }

    public void off () {
        //lights.stop();
    }

    public Command runPattern(double pattern) {
        return run(()-> setColor());
        //return run(() -> red.applyTo(lightBuffer));
    }

    public void setColor (){
        light.set(-.97);
        light.set(SmartDashboard.getNumber("light index", 0));
        //System.out.println("LED light set");
    }

    public double getColorChange(int currentIndex)
    {
        return patterns[currentIndex];
    }

    public void setColorChange(int wantedIndex)
    {
        LEDindex = wantedIndex;
    }

    @Override
    public void periodic (){
        // refresh what LEDs need to display
        //lights.setData(lightBuffer);
        setColorChange((int)(SmartDashboard.getNumber("color Index", 0)));
    }
}
