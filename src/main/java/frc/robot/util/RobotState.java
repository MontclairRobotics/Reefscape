package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;

// height should be extension of elevator in meters
public enum RobotState {
    L4 (1.2,Rotation2d.fromDegrees(-58)),
    L3 (0.45,Rotation2d.fromDegrees(-33)),
    L2 (0,Rotation2d.fromDegrees(-17)),
    L1 (0,Rotation2d.fromDegrees(-40)),
    Intake(0, Arm.MAX_ANGLE),
    Processor(0, Rotation2d.fromDegrees(0)),
    Net(0, Rotation2d.fromDegrees(0)),
    DrivingAlgae(0, Rotation2d.fromDegrees(0)),
    DrivingCoral(0, Arm.MAX_ANGLE),
    DrivingNone(0, Intake.getAngle()),
    L1Algae(0, Rotation2d.fromDegrees(0)),
    L2Algae(0, Rotation2d.fromDegrees(0));

    private double height;
    private Rotation2d angle;

    RobotState(double height, Rotation2d angle) {
        this.height = height;
        this.angle = angle;
    }

    public RobotState fromInt(int x){
        switch(x) {
            case 4: return L4;
            case 3: return L3;
            case 2: return L2;
            case 1: return L1;
        }
        return null;
    }

    public static RobotState getDefaultForPiece(GamePiece piece) {
        if (piece == GamePiece.Coral) {
            return DrivingCoral;
        } else if (piece == GamePiece.Algae) {
            return DrivingAlgae;
        } else {
            return DrivingNone;
        }
    }

    public static RobotState fromString(String s){
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

    
    public static boolean isAt(RobotState state) {
        //checks if arm is at the angle
        if(Math.abs(state.getAngle().getDegrees() - RobotContainer.arm.getEndpointAngle().getDegrees()) < 5) {
            //checks if elevator is at the height
            if(Math.abs(state.getHeight() - RobotContainer.elevator.getExtension()) < .05) {
                return true;
            }
        }
        return false;
    }

}