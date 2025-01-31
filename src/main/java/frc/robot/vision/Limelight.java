package frc.robot.vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Limelight extends SubsystemBase {

    /* CONSTANTS */
    public static final double limelightOffsetAngleVertical = 20; // TODO: set this
    public static final double limelightMountHeight = .1; // TODO: set this
    public static final double coralStationTagHeightMeters = 1.35255; // make sure these two are correct
    // does it need to be to the center of the tag?
    public static final double reefTagHeightMeters = 0.174625;
    public static final double reefOffsetFromCenterOfTag = 0;

    public static final double goalHeightReef = reefTagHeightMeters - limelightMountHeight;
    public static final double goalHeightCoralStation = coralStationTagHeightMeters - limelightMountHeight;

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

    public Limelight(String cameraName) {
        this.cameraName = cameraName;
        LimelightHelpers.SetFiducialIDFiltersOverride(cameraName, validIDs);
    }

    // might not be needed
    public static boolean isCorrectID(int[] IDs, double ID) {
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
    public double getStraightDistanceToCoralStation() {
        double distance = (coralStationTagHeightMeters - limelightMountHeight)
                / Math.tan(
                        (Math.PI / 180.0)
                                * (limelightOffsetAngleVertical + getTY()));

        return distance;
    }

    //TODO: Do we need these / check if the trig is right
    public double getStraightDistanceToReef() {
        double distance = (reefTagHeightMeters - limelightMountHeight)
                / Math.tan(
                        (Math.PI / 180.0)
                                * (limelightOffsetAngleVertical + getTY()));

        return distance;
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
    public double getStraightDistanceToTag() {
        if (hasValidTarget())
            return goalHeightReef / (Math.tan(Math.toRadians(getTY() + limelightOffsetAngleVertical)));
        return 0;
    }

    //TODO: Do we need these / check if the trig is right
    public double getStrafeDistanceToReef() {
        if (isCorrectID(reefIDs, getTagID())) {
            return (Math.tan(Math.toRadians(getTX()))) * getStraightDistanceToTag();
        }
        return 0;
    }

    public int getTagID() {
        return (int) LimelightHelpers.getFiducialID(cameraName);
    }

    public double getBotPose() {
        return LimelightHelpers.getLimelightNTDouble(cameraName, "botpose");
    }

    public void periodic() {
        // tagID = (int) Limetable.getEntry("tid").getDouble(-1);
       // poseEstimationMegatag2();
    }
}
