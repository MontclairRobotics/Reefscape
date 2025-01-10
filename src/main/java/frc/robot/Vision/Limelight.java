package frc.robot.Vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    
    private String cameraName;

    public static final String llname = "";
    public static final double limelightOffsetAngleVertical = 20; //TODO: set this
    public static final double limelightMountHeight = .1; //TODO: set this
    public static final double coralStationTagHeightMeters = 1.35255; //make sure these two are correct
    //does it need to be to the center of the tag?
    public static final double reefTagHeightMeters = 0.174625;
    public static final double reefOffsetFromCenterOfTag = 0; 

    public static final double goalHeightReef = reefTagHeightMeters - limelightMountHeight;
    public static final double goalHeightCoralStation = coralStationTagHeightMeters - limelightMountHeight;

    public static final int[] reefIDsRed = {6,7,8,9,10,11};
    public static final int[] reefIDsBlue = {17,18,19,20,21,22};
    public static final int[] reefIDs = {6,7,8,9,10,11,17,18,19,20,21,22};

    public static final int[] coralStationIDsRed = {1,2};
    public static final int[] coralStationIDsBlue = {12,13};
    public static final int[] coralStationIDs = {1,2,12,13};

    //TODO: This is how off we need to go left or right to have our grabber be aligned

    int tagCount;
    NetworkTable Limetable = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = Limetable.getEntry("ty");

    double tagID = Limetable.getEntry("tid").getDouble(-1);

    public static boolean isCorrectID(int[] IDs, double ID){
        for(int n: IDs){
            if(n == ID) return true;
        }
        return false;
    }

    public static double getTX(){
        return LimelightHelpers.getLimelightNTDouble(llname, "tx");
    } 
    public static double getTY() {
        return LimelightHelpers.getLimelightNTDouble(llname, "ty");
    } 
    public static DoubleSupplier tySupplier(){
        return () -> getTY();
    }
    public static DoubleSupplier txSupplier(){
        return () -> getTX();
    }
    public double getStraightDistanceToTag(){
        if(isTargetInView()) return goalHeightReef/(Math.tan(Math.toRadians(getTY()+limelightOffsetAngleVertical)));
        return 0;
    }
    public double getStrafeDistanceToReef(){
        if(isCorrectID(reefIDs, tagID)){
            return (Math.tan(Math.toRadians(getTX()))) * getStraightDistanceToTag();
        }
        return 0;
    }
    public double getBotPose(){
        return LimelightHelpers.getLimelightNTDouble(llname, "botpose");
    }  
    public static void takeSnapshot() {
        LimelightHelpers.takeSnapshot("","Limelight Snapshot");
    }
    public boolean isTargetInView(){
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate(llname, "botpose", false);
        if(limelightMeasurement.tagCount > 0) return true;
        return false;
    }

    public void periodic(){
        tagID = Limetable.getEntry("tid").getDouble(-1);

    }
}
