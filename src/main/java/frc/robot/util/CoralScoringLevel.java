package frc.robot.util;

public enum CoralScoringLevel {
    L4,
    L3,
    L2,
    L1;

    public CoralScoringLevel fromInt(int x){
        switch(x) {
            case 4: return L4;
            case 3: return L3;
            case 2: return L2;
            case 1: return L1;
        }
        return null;
    }

    public CoralScoringLevel fromString(String s){
        switch(s) {
            case "4": return L4;
            case "3": return L3;
            case "2": return L2;
            case "1": return L1;
        }
        return null;
    }

    // public double getHeight(CoralScoringLevel level) {
    //     if(level == L4) return 0;
    //     if(level == L3) return 0;
    //     if(level == L2) return 0;
    //     if(level == L1) return 0;
    //     return 0;
    // }

}