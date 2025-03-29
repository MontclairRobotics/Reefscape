package frc.robot.util;

import edu.wpi.first.wpilibj.util.Color;


public enum GamePiece {
 
    Coral(Color.kGreen),
    Algae(Color.kTeal),
    None(Color.kFirstBlue);

    private Color color;

    GamePiece(Color color) {
        this.color = color;
    }


    public Color getColor() {
        return color;
    }
}
