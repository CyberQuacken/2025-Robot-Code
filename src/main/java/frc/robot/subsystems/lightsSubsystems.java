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

    ;

    LEDPattern red = LEDPattern.solid(Color.kRed);
    LEDPattern orange = LEDPattern.solid(Color.kOrange);
    LEDPattern yellow = LEDPattern.solid(Color.kYellow);
    LEDPattern green = LEDPattern.solid(Color.kGreen);
    LEDPattern lightBlue = LEDPattern.solid(Color.kLightBlue);
    LEDPattern darkBlue = LEDPattern.solid(Color.kDarkBlue);
    LEDPattern purple = LEDPattern.solid(Color.kPurple);
    LEDPattern magenta = LEDPattern.solid(Color.kMagenta);
    LEDPattern white = LEDPattern.solid(Color.kWhite);

    private LEDPattern[] patterns = {red, orange, yellow, green, lightBlue, darkBlue, purple, magenta, white};
    
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

    public Command runPattern(LEDPattern pattern) {
        return run(()-> setColor());
        //return run(() -> red.applyTo(lightBuffer));
    }

    public void setColor (){
        light.set(SmartDashboard.getNumber("light index", 0));
        System.out.println("LED light set");
    }

    public LEDPattern getColorChange(int currentIndex)
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
