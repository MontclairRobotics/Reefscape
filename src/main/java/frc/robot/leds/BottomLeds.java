package frc.robot.leds;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;

public class BottomLeds {

    public static final int PORT = -1;
    public static final int LENGTH = 20;
    AddressableLED led;
    AddressableLEDBuffer ledBuffer;


    public BottomLeds() {
        led = new AddressableLED(PORT);
        ledBuffer = new AddressableLEDBuffer(LENGTH);
        led.setData(ledBuffer);
        led.start();

    }

    public void playPattern(LEDPattern pattern){
        pattern.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }
}
