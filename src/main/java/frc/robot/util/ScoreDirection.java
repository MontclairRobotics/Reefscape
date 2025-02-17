package frc.robot.util;

public enum ScoreDirection {
    LEFT(0,0),
    RIGHT(0,0),
    CENTER(0,0);

    private double txTargetError;
    private double tyTargetError;

    private ScoreDirection(double txTargetError, double tyTargetError) {
        this.txTargetError = txTargetError;
        this.tyTargetError = tyTargetError;
    }
    
    public double getTxTargetError() {
        return txTargetError;
    }

    public double getTyTargetError() {
        return tyTargetError;
    }

    public boolean isLeft() {
        return this == ScoreDirection.LEFT;
    }

    public boolean isRight() {
        return this == ScoreDirection.RIGHT;
    }
}
