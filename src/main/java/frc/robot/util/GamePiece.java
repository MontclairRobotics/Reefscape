package frc.robot.util;

import edu.wpi.first.wpilibj.util.Color;

//TODO instead of colors, perhaps this should take in LED patterns
public enum GamePiece {
    Coral(Color.kWhite),
    Algae(Color.kSeaGreen),
    None(Color.kBeige); // TODO this should be alliance color?

    private Color color;

    GamePiece(Color color) {

        this.color = color;
    }


    public Color getColor() {
        return color;
    }
}
