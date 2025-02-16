package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

// height should be extension of elevator in meters
public enum ArmPosition {
    L4 (1.15,Rotation2d.fromDegrees(-57)),
    L3 (0.424,Rotation2d.fromDegrees(-33)),
    L2 (0,Rotation2d.fromDegrees(-17)),
    L1 (0,Rotation2d.fromDegrees(0)),
    Intake(0, Rotation2d.fromDegrees(70)),
    Processor(0, Rotation2d.fromDegrees(0)),
    Net(0, Rotation2d.fromDegrees(0)),
    DrivingAlgae(0, Rotation2d.fromDegrees(0)),
    DrivingCoral(0, Rotation2d.fromDegrees(0)),
    DrivingNone(0, Rotation2d.fromDegrees(0)),
    L1Algae(0, Rotation2d.fromDegrees(0)),
    L2Algae(0, Rotation2d.fromDegrees(0));

    private double height;
    private Rotation2d angle;

    ArmPosition(double height, Rotation2d angle) {
        this.height = height;
        this.angle = angle;
    }

    public ArmPosition fromInt(int x){
        switch(x) {
            case 4: return L4;
            case 3: return L3;
            case 2: return L2;
            case 1: return L1;
        }
        return null;
    }

    public static ArmPosition getDefaultForPiece(GamePiece piece) {
        if (piece == GamePiece.Coral) {
            return DrivingCoral;
        } else if (piece == GamePiece.Algae) {
            return DrivingAlgae;
        } else {
            return DrivingNone;
        }
    }

    public static ArmPosition fromString(String s){
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

    public Rotation2d getAngle() {
        return angle;
    }

}