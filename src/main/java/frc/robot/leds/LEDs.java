package frc.robot.leds;

import frc.robot.util.GamePiece;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;

public class LEDs extends SubsystemBase {
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
        // static LEDPattern m_scrollingRainbowProgress = m_progressBar.scrollingRainbowProgress();
    public LEDs() {
            led = new AddressableLED(PORT);
            led.setLength(LENGTH);
            ledBuffer = new AddressableLEDBuffer(LENGTH);
            led.start();
        }
    // Usually Returns Blinking Synched with RSL, Currently Not For Testing Purposes 
    public static LEDPattern holding(Color color) {
        LEDPattern object = LEDPattern.solid(color);
        //LEDPattern blinkingObj = object.synchronizedBlink(RobotController::getRSLState);
        LEDPattern blinkingObj = object.blink(Seconds.of(0.1));
        return blinkingObj;
    }

    // public static LEDPattern shot(Color color) {
    //     LEDPattern shotGamePiece;
    //     if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
    //         shotGamePiece = LEDPattern.gradient(GradientType.kContinuous,Color.kFirstRed, color);
    //     } else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
    //         shotGamePiece = LEDPattern.gradient(GradientType.kContinuous,Color.kFirstBlue, color);
    //     } else {
    //         Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, 0.5, Color.kBlack);
    //         LEDPattern base = LEDPattern.rainbow(255, 255);
    //         LEDPattern mask = LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(0.25));
    //         shotGamePiece = base.mask(mask);
    //     }
    //     return shotGamePiece;
    // }
    public static LEDPattern breathingPattern() {
        LEDPattern base;
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            base = LEDPattern.gradient(GradientType.kDiscontinuous,Color.kFirstRed, Color.kDarkRed);
        } else if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ) {
            base = LEDPattern.gradient(GradientType.kDiscontinuous,Color.kFirstBlue, Color.kDarkBlue);
        } else {
            base = LEDPattern.solid(Color.kWhite);
        }
        LEDPattern pattern =  base.breathe(Seconds.of(2.5));
        return pattern;
    }
    public static LEDPattern progress(){
        LEDPattern base;
        if(DriverStation.getAlliance().isPresent()){
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                base = LEDPattern.solid(Color.kFirstRed);
            } else {
                base = LEDPattern.solid(Color.kFirstBlue);
        }
        } else {
            base = LEDPattern.solid(Color.kWhite);
    }
        LEDPattern scrollingBase = base.scrollAtAbsoluteSpeed(Meter.per(Second).of(1.5), kLedSpacing);
        LEDPattern m_progress = LEDPattern.progressMaskLayer(() -> RobotContainer.elevator.getHeight() / Elevator.MAX_HEIGHT);
        LEDPattern basedProgress = scrollingBase.mask(m_progress);
        return basedProgress;  
    }   
    public Command playPatternCommand(LEDPattern pattern) {
        return Commands.run(() -> pattern.applyTo(ledBuffer), this).ignoringDisable(true);
    }
    public void periodic(){
        led.setData(ledBuffer); 
    }
}