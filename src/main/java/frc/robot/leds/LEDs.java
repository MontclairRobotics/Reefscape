package frc.robot.leds;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
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
    public static final int LENGTH =  100;
    public static final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
    public static final Distance kLedSpacing = Units.Meters.of(1 / 100.0);
    public static final LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(1), kLedSpacing);
    // public static final LEDPattern m_progressbar = LEDs.progressBar(Color.kRed);
    static AddressableLED led;
    static AddressableLEDBuffer ledBuffer;
    static LEDPattern pattern;
        
        // static LEDPattern m_scrollingRainbowProgress = m_progressBar.scrollingRainbowProgress();
    public LEDs() {
            led = new AddressableLED(PORT);
            led.setLength(LENGTH);
            ledBuffer = new AddressableLEDBuffer(LENGTH);
            led.start();
        }
    public static void scrollingRainbowProgress() {
        
    }

    public static LEDPattern holding() {
        LEDPattern objectInArm = LEDPattern.solid(Color.kBlue);
        return objectInArm;
    }

    public static LEDPattern shot() {
        LEDPattern shotGamPiece = LEDPattern.solid(Color.kGreen);
        LEDPattern pattern = shotGamPiece.blink(Units.Seconds.of(1.5));
        return pattern;
    }

    // TODO This isn't implemented properly, you're returning the contents of the instance variable
    // pattern, not the contants of the local variable created here
    public static LEDPattern breathingPattern(boolean onRed, boolean startUp) {
        LEDPattern base;
        boolean noAlliance = false;
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            base = LEDPattern.gradient(GradientType.kDiscontinuous,Color.kRed, Color.kDarkRed);
        } else if(DriverStation.getAlliance().isPresent()) {
            base = LEDPattern.gradient(GradientType.kDiscontinuous,Color.kBlue, Color.kDarkBlue);
        } else {
            base = LEDPattern.solid(Color.kFirstRed);
        }
        if(! noAlliance) {
            LEDPattern pattern = base.breathe(Units.Seconds.of(5));
        } else if(startUp) {
            LEDPattern pattern = base.blink(Units.Second.of(0.3));
        }
        
        return pattern;
    }
    
    public static LEDPattern progress(){
        LEDPattern base = LEDPattern.rainbow(255,255);
        LEDPattern m_progress = LEDPattern.progressMaskLayer(() -> RobotContainer.elevator.getHeight() / Elevator.MAX_HEIGHT);
        return m_progress;
    }
    public static Command runPattern(LEDPattern pattern){
        return Commands.run(() -> pattern.applyTo(ledBuffer), RobotContainer.ledControl);
    }

    public void periodic(){
        pattern.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }
}
