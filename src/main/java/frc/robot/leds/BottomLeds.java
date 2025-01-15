package frc.robot.leds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BottomLeds extends SubsystemBase{
    
    public static final int kPORT = -1;
    public static final int kLENGTH = -1;
      
  private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);

  private static final Distance kLedSpacing = Units.Meters.of(1 / 120.0);
  
  private final LEDPattern m_scrollingRainbow =
      m_rainbow.scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(1), kLedSpacing);
    AddressableLED led;
    AddressableLEDBuffer ledBuffer;


    public BottomLeds() {

        led = new AddressableLED(kPORT);
        ledBuffer = new AddressableLEDBuffer(kLENGTH);
        led.setData(ledBuffer);
        led.start();


    }
    AddressableLEDBufferView m_left = ledBuffer.createView(0, 59);

    // The section of the strip on the right side of the robot.
    // This section spans LEDs from index 60 through index 119, inclusive.
    // This view is reversed to cancel out the serpentine arrangement of the
    // physical LED strip on the robot.
    AddressableLEDBufferView m_right = ledBuffer.createView(60, 119).reversed();
    
    public void playPattern(LEDPattern pattern){
        pattern.applyTo(ledBuffer);
        led.setData(ledBuffer);

    final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);

    final Distance kLedSpacing = Units.Meters.of(1 / 120.0);

    final LEDPattern m_scrollingRainbow =
        m_rainbow.scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(1), kLedSpacing);
    }

    public void robotPeriodic() {
        // Update the buffer with the rainbow animation
        m_scrollingRainbow.applyTo(ledBuffer);
        // Set the LEDs
        led.setData(ledBuffer);
    }
}