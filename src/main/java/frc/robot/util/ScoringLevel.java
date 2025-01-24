package frc.robot.util;

public enum ScoringLevel {
    L4 (0),
    L3 (0),
    L2 (0),
    L1 (0);

    private double height;

    ScoringLevel(double height) {
        this.height = height;
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

    public ScoringLevel fromString(String s){
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

}