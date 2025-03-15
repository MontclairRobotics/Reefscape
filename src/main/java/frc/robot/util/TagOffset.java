package frc.robot.util;

import edu.wpi.first.math.util.Units;

public enum TagOffset {
    LEFT(-5.5, 10.5, Units.inchesToMeters(35.5/2), 0.165), //TODO: SET
    RIGHT(5.5, 10.5, Units.inchesToMeters(35.5/2), -0.165),
    CENTER(0, 10.5, Units.inchesToMeters(35.5/2), 0),
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

    public double getYOffsetM() {
        return yOffsetM;
    }

    public double getXOffsetM() {
        return xOffsetM;
    }

    public boolean isLeft() {
        return this == TagOffset.LEFT;
    }

    public boolean isRight() {
        return this == TagOffset.RIGHT;
    }
}
