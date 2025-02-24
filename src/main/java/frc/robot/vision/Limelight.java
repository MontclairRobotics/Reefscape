package frc.robot.vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.vision.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {

    /* CONSTANTS */
    public static final double coralStationTagHeightMeters = 1.35255; // make sure these two are correct
    // does it need to be to the center of the tag?
    public static final double reefTagHeightMeters = 0.174625;
    public static final double reefOffsetFromCenterOfTag = 0;

    public static final int[] reefIDsRed = { 6, 7, 8, 9, 10, 11 };
    public static final int[] reefIDsBlue = { 17, 18, 19, 20, 21, 22 };
    public static final int[] reefIDs = { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };

    public static final int[] coralStationIDsRed = { 1, 2 };
    public static final int[] coralStationIDsBlue = { 12, 13 };
    public static final int[] coralStationIDs = { 1, 2, 12, 13 };

    public static final double TARGET_DEBOUNCE_TIME = 0.2;

    /* INSTANCE VARIABLES */
    private int tagCount;
    private int[] validIDs = {}; // TODO: set these
    private String cameraName;
    private Debouncer targetDebouncer = new Debouncer(TARGET_DEBOUNCE_TIME, DebounceType.kFalling);
    private boolean shouldRejectUpdate;
    private LimelightHelpers.PoseEstimate mt2;

    private double angleVelocityTolerance = 540 * Math.PI / 180; // in radians per sec

    private double cameraHeightMeters;
    private double cameraAngle;
    private double cameraOffsetX; // right is positive
    private double cameraOffsetY; //forward is positive

    private StructPublisher<Pose2d> posePublisher;

    public Limelight(String cameraName, double cameraHeightMeters, double cameraAngle, double cameraOffsetX, double cameraOffsetY) {
        this.cameraName = cameraName;
        this.cameraHeightMeters = cameraHeightMeters;
        this.cameraAngle = cameraAngle;
        this.cameraOffsetX = cameraOffsetX;
        this.cameraOffsetY = cameraOffsetY;
        LimelightHelpers.SetFiducialIDFiltersOverride(cameraName, validIDs);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable limelightTable = inst.getTable(cameraName);
        posePublisher = limelightTable.getStructTopic("Pose", Pose2d.struct).publish();
    }

    // might not be needed
    public static boolean isCorrectID(int ID, int... IDs) {
        for (int n : IDs) {
            if (n == ID)
                return true;
        }
        return false;
    }

    // from last years robot
    public double getTimestampSeconds() {
        double latency = (LimelightHelpers.getLimelightNTDouble(cameraName, "cl")
                + LimelightHelpers.getLimelightNTDouble(cameraName, "tl"))
                / 1000.0;

        return Timer.getFPGATimestamp() - latency;
    }

    // from last years robot as well
    public boolean hasValidTarget() {
        boolean hasMatch = (LimelightHelpers.getLimelightNTDouble(cameraName, "tv") == 1.0);
        return targetDebouncer.calculate(hasMatch);
    }

    // public void poseEstimationMegatag2() {

    //     // TODO does the angle need to be wrapped between 0 and 360
    //     LimelightHelpers.SetRobotOrientation(cameraName, RobotContainer.drivetrain.getWrappedHeading().getDegrees(), 0, 0, 0, 0, 0);
    //     mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
        
    //     //TODO simulate limelight properly
    //     if (mt2 != null) { 
    //         if (mt2.tagCount == 0) {
    //             //rejects current measurement if there are no aprilTags
    //             shouldRejectUpdate = true;
    //         }
    //         if (Math.abs(RobotContainer.drivetrain.getCurrentSpeeds().omegaRadiansPerSecond) > angleVelocityTolerance) {
    //             shouldRejectUpdate = true;
    //         }
    //         //adds vision measurement if conditions are met
    //         if (!shouldRejectUpdate) {
    //             // RobotContainer.drivetrain.swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
    //             RobotContainer.drivetrain.addVisionMeasurement(
    //                     mt2.pose,
    //                     mt2.timestampSeconds);
    //         }
    //     }
    // }

    //TODO: Do we need these / check if the trig is right
    

    public double getDistanceToTag(double tagHeightMeters) {
        if (hasValidTarget()) {
            double distance = (tagHeightMeters - cameraHeightMeters)
                    / Math.tan(
                            (Math.PI / 180.0)
                                    * (cameraAngle + getTY()));
            return distance;
        }
        return 0;
    }

    public double getStraightDistanceToTag(double tagHeightMeters) {
        if (hasValidTarget()) {
            double distance = getDistanceToTag(tagHeightMeters);
            distance = distance / Math.cos(getTX() * (Math.PI / 180.0));
            return distance + cameraOffsetY;
        }
        return 0;
    }

    public double getHorizontalDistanceToTag(double tagHeightMeters) {
        double distance = getDistanceToTag(tagHeightMeters);
        try {
            distance = distance / Math.sin(getTX() * (Math.PI / 180.0));
        } catch (Exception e) {
            e.printStackTrace();
            distance = 0;
        }
        return distance + cameraOffsetX;
    }

    public double getDistanceToCoralStation() {
        return getDistanceToTag(coralStationTagHeightMeters);
    }

    public double getStraightDistanceToCoralStation() {
        return getStraightDistanceToTag(coralStationTagHeightMeters);

    }

    public double getHorizontalDistanceToCoralStation() {
        return getHorizontalDistanceToTag(coralStationTagHeightMeters);
    }

    // ISN'T OFFSET FOR THE CENTER OF THE ROBOT!!!!!!!
    public double getDistanceToReef() {
        return getDistanceToTag(reefTagHeightMeters);
    }
   
    //TODO: Do we need these / check if the trig is right
    public double getStraightDistanceToReef() {
        return getStraightDistanceToTag(reefTagHeightMeters);
    }


    public double getHorizontalDistanceToReef() {
        return getHorizontalDistanceToTag(reefTagHeightMeters);
    }

    public double getTX() {
        return LimelightHelpers.getTX(cameraName);
    }

    public double getTY() {
        return LimelightHelpers.getTY(cameraName);
    }

    public DoubleSupplier tySupplier() {
        return () -> getTY();
    }

    public DoubleSupplier txSupplier() {
        return () -> getTX();
    }

    //TODO: Do we need these / check if the trig is right
    // public double getStraightDistanceToTag() {
    //     if (hasValidTarget())
    //         return goalHeightReef / (Math.tan(Math.toRadians(getTY() + limelightOffsetAngleVertical)));
    //     return 0;
    // }

    //TODO: Do we need these / check if the trig is right
    public double getStrafeDistanceToReef() {
        if (isCorrectID(getTagID(), reefIDs)) {
            return (Math.tan(Math.toRadians(getTX()))) * getStraightDistanceToReef();
        }
        return 0;
    }

    public int getTagID() {
        return (int) LimelightHelpers.getFiducialID(cameraName);
    }

    public void periodic() {
        // tagID = (int) Limetable.getEntry("tid").getDouble(-1);
        // poseEstimationMegatag2();
        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
        if (poseEstimate != null && poseEstimate.pose != null) {
            posePublisher.set(poseEstimate.pose);
        }
    }

    public Command ifHasTarget(Command cmd) {
        return cmd.onlyWhile(this::hasValidTarget);
    }

    /**
     * Returns the transform from the robot to the camera
     * @return
     */
    public Transform3d getRobotToCamera() {
        Translation3d robotToCameraTrl = new Translation3d(cameraOffsetX, cameraOffsetY, cameraHeightMeters);
        Rotation3d robotToCameraRot = new Rotation3d(0, Units.degreesToRadians(cameraAngle), 0);
        // return new Transform3d(robotToCameraTrl, Rotation3d.kZero);
        return new Transform3d(robotToCameraTrl, robotToCameraRot);
    }
}
