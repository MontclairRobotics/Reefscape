package frc.robot.util;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.leds.LEDs;

public enum GamePiece {
    // TODO: Use This?
    Coral(LEDs.shot(Color.kSkyBlue)),
    Algae(LEDs.shot(Color.kWhite)),
    None(LEDPattern.gradient(GradientType.kDiscontinuous, Color.kFirstBlue,Color.kFirstRed));

    private LEDPattern pattern;

    GamePiece(LEDPattern pattern) {
        this.pattern = pattern;
    }


    public LEDPattern getPattern() {
        return pattern;
    }
}
