package frc.robot.util;

public enum ScoringLevel {
    L4 (0,0),
    L3 (0,0),
    L2 (0,0),
    L1 (0,0);

    private double height;
    private double angle;

    ScoringLevel(double height, double angle) {
        this.height = height;
        this.angle = angle;
    }

    public ScoringLevel fromInt(int x){
        switch(x) {
            case 4: return L4;
            case 3: return L3;
            case 2: return L2;
            case 1: return L1;
        }
        return null;
    }

    public static ScoringLevel fromString(String s){
        switch(s) {
            case "4": return L4;
            case "3": return L3;
            case "2": return L2;
            case "1": return L1;
        }
        return null;
    }

    public double getHeight() {
        return height;
    }

    public double getAngle() {
        return angle;
    }

}