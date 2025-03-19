package frc.robot.util;

public enum TagOffset {
    LEFT(0, 0, -0.165, 0.5), //TODO: SET
    RIGHT(0, 0, 0.165, 0.5),
    CENTER(0, 0, 0, 0),
    LEFT_INTAKE(0, 0, 0, 0),
    RIGHT_INTAKE(0, 0, 0, 0),
    CENTER_INTAKE(0, 0, 0, 0);

    private double txTargetError;
    private double tyTargetError;
    private double xOffsetM;
    private double yOffsetM;

    private TagOffset(double txTargetError, double tyTargetError, double xOffsetM, double yOffsetM) {
        this.txTargetError = txTargetError;
        this.tyTargetError = tyTargetError;
        this.xOffsetM = xOffsetM;
        this.yOffsetM = yOffsetM;
    }

    public double getTxTargetError() {
        return txTargetError;
    }

    public double getTyTargetError() {
        return tyTargetError;
    }

    public double getForwardOffsetM() {
        return yOffsetM;
    }

    public double getHorizontalOffsetM() {
        return xOffsetM;
    }

    public boolean isLeft() {
        return this == TagOffset.LEFT;
    }

    public boolean isRight() {
        return this == TagOffset.RIGHT;
    }
}
