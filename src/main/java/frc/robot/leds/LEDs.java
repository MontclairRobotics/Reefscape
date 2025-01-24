package frc.robot.leds;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;

public class LEDs extends SubsystemBase{
    public static final int kPORT = -1;
    public static final int kLENGTH = -1; 
    public static final int PORT = 9;
    public static final int LENGTH = 100;
    public static final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
    public static final Distance kLedSpacing = Units.Meters.of(1 / 100.0);
    public static final LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(1), kLedSpacing);
    // public static final LEDPattern m_progressbar = LEDs.progressBar(Color.kRed);
    static AddressableLED led;
    static AddressableLEDBuffer ledBuffer;
    static LEDPattern pattern;
        
        // static LEDPattern m_scrollingRainbowProgress = m_progressBar.scrollingRainbowProgress();
    public LEDs(){
            led = new AddressableLED(PORT);
            led.setLength(LENGTH);
            ledBuffer = new AddressableLEDBuffer(LENGTH);
            led.start();
        }
        public static void progressBar(Color color) {
        LEDPattern base = LEDPattern.solid(color);
        LEDPattern mask = LEDPattern.progressMaskLayer(() -> RobotContainer.elevator.getHeight() /
        Elevator.ELEVATOR_MAX_HEIGHT);
        pattern = base.mask(mask);
        pattern.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }
    public static void scrollingRainbowProgress() {
        LEDPattern mask = LEDPattern.progressMaskLayer(() -> RobotContainer.elevator.getHeight() / Elevator.ELEVATOR_MAX_HEIGHT);
        pattern = m_rainbow.mask(mask);
        pattern.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }

    public static Command runPattern(LEDPattern pattern){
        return Commands.run(() -> pattern.applyTo(ledBuffer), RobotContainer.BottomLEDs);
    }

    public void periodic(){
        pattern.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }
}
