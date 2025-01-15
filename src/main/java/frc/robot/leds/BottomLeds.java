package frc.robot.leds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
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
        private static final int kPort = 9;
        private static final int kLength = 120;

        private final AddressableLED m_led;
        private final AddressableLEDBuffer m_buffer;


    }
    AddressableLEDBufferView m_left = m_buffer.createView(0, 59);

    // The section of the strip on the right side of the robot.
    // This section spans LEDs from index 60 through index 119, inclusive.
    // This view is reversed to cancel out the serpentine arrangement of the
    // physical LED strip on the robot.
    AddressableLEDBufferView m_right = m_buffer.createView(60, 119).reversed();
    
    public void playPattern(LEDPattern pattern){
        pattern.applyTo(ledBuffer);
        led.setData(ledBuffer);
        // all hues at maximum saturation and half brightness
    private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);

    // Our LED strip has a density of 120 LEDs per meter
    private static final Distance kLedSpacing = Meters.of(1 / 120.0);

    // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a speed
    // of 1 meter per second.
    private final LEDPattern m_scrollingRainbow =
        m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
    }
    public void robotPeriodic() {
        // Update the buffer with the rainbow animation
        m_scrollingRainbow.applyTo(m_ledBuffer);
        // Set the LEDs
        m_led.setData(m_ledBuffer);
    }
    
}
