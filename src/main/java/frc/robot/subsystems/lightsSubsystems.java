package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class lightsSubsystems extends SubsystemBase{

    private AddressableLED lights;
    private AddressableLEDBuffer lightBuffer;

    private LEDPattern[] patterns;
    
    LEDPattern red = LEDPattern.solid(Color.kRed);
    LEDPattern orange = LEDPattern.solid(Color.kOrange);
    LEDPattern yellow = LEDPattern.solid(Color.kYellow);
    LEDPattern green = LEDPattern.solid(Color.kGreen);
    LEDPattern lightBlue = LEDPattern.solid(Color.kLightBlue);
    LEDPattern darkBlue = LEDPattern.solid(Color.kDarkBlue);
    LEDPattern purple = LEDPattern.solid(Color.kPurple);
    LEDPattern magenta = LEDPattern.solid(Color.kMagenta);
    LEDPattern white = LEDPattern.solid(Color.kWhite);

    public lightsSubsystems(int port, int length, LEDPattern[] setpatterns) {
        lights = new AddressableLED(port);
        lights.setLength(length);
        patterns = setpatterns;

        lightBuffer = new AddressableLEDBuffer(length);

    }

    public void on() {
        lights.start();
        // I believe that black just means not on for the lights
        setDefaultCommand(runPattern(patterns[0]).withName("Off"));
    }

    public void off () {
        lights.stop();
    }

    public Command runPattern(LEDPattern pattern) {
        return run(() -> pattern.applyTo(lightBuffer));
    }

    @Override
    public void periodic (){
        // refresh what LEDs need to display
        lights.setData(lightBuffer);
    }
}
