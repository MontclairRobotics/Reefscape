package frc.robot.leds;

import frc.robot.util.GamePiece;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.Map;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;

public class LEDs extends SubsystemBase {
    public static final int PORT = 2;
    public static final int LENGTH = 61;
    public static final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
    public static final Distance kLedSpacing = Units.Meters.of(1 / 100.0);
    public static final LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(0.2),
            kLedSpacing);
    private static LEDPattern alliancePattern = alliancePattern();
    private static LEDPattern disabledAlliancePattern = disabledAlliancePattern();
    private static Alliance prevAlliance;
    // public static final LEDPattern m_progressbar = LEDs.progressBar(Color.kRed);

    private LEDPattern altPattern;
    private Timer altTimer = new Timer();
    private double altTime = 0;
    static AddressableLED led;
    static AddressableLEDBuffer ledBuffer;

    // static LEDPattern m_scrollingRainbowProgress =
    // m_progressBar.scrollingRainbowProgress();
    public LEDs() {
        led = new AddressableLED(PORT);
        led.setLength(LENGTH);
        ledBuffer = new AddressableLEDBuffer(LENGTH);
        led.start();

    }

    // Usually Returns Blinking Synched with RSL, Currently Not For Testing Purposes
    public static LEDPattern blink(Color color) {
        LEDPattern object = LEDPattern.solid(color);
        // LEDPattern blinkingObj =
        // object.synchronizedBlink(RobotController::getRSLState);
        LEDPattern blinkingObj = object.blink(Seconds.of(0.1));
        return blinkingObj;
    }

    // Usually Returns Blinking Synched with RSL, Currently Not For Testing Purposes
    public static LEDPattern holding(Color color) {
        LEDPattern pattern = LEDPattern.solid(color);
        // LEDPattern blinkingObj =
        // object.synchronizedBlink(RobotController::getRSLState);
        // LEDPattern blinkingObj = object.blink(Seconds.of(0.1));
        return pattern;
    }

    // public static LEDPattern shot(Color color) {
    // LEDPattern shotGamePiece;
    // if (DriverStation.getAlliance().isPresent() &&
    // DriverStation.getAlliance().get() == Alliance.Red){
    // shotGamePiece = LEDPattern.gradient(GradientType.kContinuous,Color.kFirstRed,
    // color);
    // } else if (DriverStation.getAlliance().isPresent() &&
    // DriverStation.getAlliance().get() == Alliance.Blue) {
    // shotGamePiece =
    // LEDPattern.gradient(GradientType.kContinuous,Color.kFirstBlue, color);
    // } else {
    // Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, 0.5, Color.kBlack);
    // LEDPattern base = LEDPattern.rainbow(255, 255);
    // LEDPattern mask =
    // LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(0.25));
    // shotGamePiece = base.mask(mask);
    // }
    // return shotGamePiece;
    // }
    public static LEDPattern alliancePattern() {
        LEDPattern base;
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            base = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kDarkRed, Color.kFirstRed);
        } else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            base = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kFirstBlue, Color.kDarkBlue);
        } else {
            base = LEDPattern.solid(Color.kWhite);
        }
        return base;
    }

    public static LEDPattern disabledAlliancePattern() {
        LEDPattern base;
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            base = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kFirstRed, Color.kDarkRed);
        } else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            base = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kFirstBlue, Color.kDarkBlue);
        } else {
            base = LEDPattern.solid(Color.kWhite);
        }
        LEDPattern pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(75));
        return pattern;
    }

    public Command playLEDPatternCommand(LEDPattern pattern, double seconds) {
        return Commands.runOnce(() -> {
            altPattern = pattern;
            altTime = seconds;
            altTimer.restart();
        }, this);
    }

    public void playLEDPattern(LEDPattern pattern, double seconds) {
        altPattern = pattern;
        altTime = seconds;
        altTimer.restart();
    }

    public Command stopAltPattern() {
        return Commands.runOnce(() -> {
            altTimer.stop();
        });
    }

    public Command getDefaultCommand() {
        return Commands.run(() -> {
            LEDPattern pattern;
            if (altTimer.hasElapsed(altTime)) {
                altTimer.stop();
            }
            if (altTimer.isRunning()) {
                pattern = altPattern;
            } else if (RobotContainer.elevator.isVelociatated()) {
                pattern = progress();
            } else if (RobotContainer.rollers.getHeldPiece() != GamePiece.None && DriverStation.isEnabled()) {
                pattern = holding(RobotContainer.rollers.getHeldPiece().getColor());
            } else if (DriverStation.isDisabled()) {
                pattern = disabledAlliancePattern;
            } else {
                pattern = alliancePattern;
            }
            pattern.applyTo(ledBuffer);
        }, this).ignoringDisable(true);
    }

    public static LEDPattern progress() {
        LEDPattern base;
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                base = LEDPattern.solid(Color.kFirstRed);
            } else {
                base = LEDPattern.solid(Color.kFirstBlue);
            }
        } else {
            base = LEDPattern.solid(Color.kWhite);
        }
        LEDPattern scrollingBase = base.scrollAtAbsoluteSpeed(Meter.per(Second).of(1.5), kLedSpacing);
        LEDPattern m_progress = LEDPattern
                .progressMaskLayer(() -> RobotContainer.elevator.getHeight() / Elevator.MAX_HEIGHT);
        LEDPattern basedProgress = scrollingBase.mask(m_progress);
        return basedProgress;
    }

    public Command playPatternCommand(LEDPattern pattern) {
        return Commands.runOnce(() -> pattern.applyTo(ledBuffer), this).ignoringDisable(true);
    }

    public void periodic() {
        if (DriverStation.isDisabled()) {
            Alliance alliance = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue);
            if (alliance != prevAlliance || prevAlliance == null) {
                System.out.println("Switchign");
                prevAlliance = alliance;
                alliancePattern = alliancePattern();
                disabledAlliancePattern = disabledAlliancePattern();
            }
        }
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            int red = ledBuffer.getRed(i);
            int green = ledBuffer.getGreen(i);
            int blue = ledBuffer.getBlue(i);
            ledBuffer.setRGB(i, green, red, blue); //RGB -> GBR
        }
        led.setData(ledBuffer);
    }
}