package frc.robot.leds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator;

public class BottomLEDs extends SubsystemBase{
    
    public static final int kPORT = -1;
    public static final int kLENGTH = -1;
      
  private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);

  private static final Distance kLedSpacing = Units.Meters.of(1 / 120.0);
  private static Elevator elevator = new Elevator();
  
  private final LEDPattern m_scrollingRainbow =
    m_rainbow.scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(1), kLedSpacing);
    AddressableLED led;
    AddressableLEDBuffer ledBuffer;

    public BottomLEDs() {

        led = new AddressableLED(kPORT);
        ledBuffer = new AddressableLEDBuffer(kLENGTH);
        led.setData(ledBuffer);
        led.start();

        setDefaultCommand(playPatternCommand(LEDPattern.solid(Color.kBlack)).withName("Off"));
    }
    AddressableLEDBufferView m_left = ledBuffer.createView(0, 59);

    // The section of the strip on the right side of the robot.
    // This section spans LEDs from index 60 through index 119, inclusive.
    // This view is reversed to cancel out the serpentine arrangement of the
    // physical LED strip on the robot.
    AddressableLEDBufferView m_right = ledBuffer.createView(60, 119).reversed();
    
    public void rainbowPattern(LEDPattern pattern){
        pattern.applyTo(ledBuffer);
        led.setData(ledBuffer);
        final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
        final Distance kLedSpacing = Units.Meters.of(1 / 120.0);
        final LEDPattern scrollingRainbow =
            m_rainbow.scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(1), kLedSpacing);
    }
    public void progressBar(){
        LEDPattern pattern = LEDPattern.progressMaskLayer(() -> elevator.getHeight() / Elevator.getMaxHeight());;
    }
    public void robotPeriodic() {
        playPatternCommand(m_scrollingRainbow);
        led.setData(ledBuffer);
    }
    public Command playPatternCommand(LEDPattern pattern){
        return run(()-> pattern.applyTo(ledBuffer));
    }
}