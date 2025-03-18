package frc.robot.vision;

import java.util.Set;
import java.util.HashMap;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.PoseUtils;
import frc.robot.vision.LimelightHelpers.RawFiducial;

public class Limelight extends SubsystemBase {

    /* CONSTANTS */
    public static final double coralStationTagHeightMeters = 1.35255; // make sure these two are correct
    // does it need to be to the center of the tag?
    public static final double reefTagHeightMeters = //0.174625; 
    0.3;
    public static final double reefOffsetFromCenterOfTag = 0;

    public static final int[] reefIDsRed = { 6, 7, 8, 9, 10, 11 };
    public static final int[] reefIDsBlue = { 17, 18, 19, 20, 21, 22 };
    public static final int[] reefIDs = { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };

    public static final int[] coralStationIDsRed = { 1, 2 };
    public static final int[] coralStationIDsBlue = { 12, 13 };
    public static final int[] coralStationIDs = { 1, 2, 12, 13 };
    public static HashMap<Integer, Rotation2d> tagRotationsMap = new HashMap<Integer, Rotation2d>();
    {
        tagRotationsMap.put(6, Rotation2d.fromDegrees(120));
        tagRotationsMap.put(7, Rotation2d.fromDegrees(180));
        tagRotationsMap.put(8, Rotation2d.fromDegrees(-120));
        tagRotationsMap.put(9, Rotation2d.fromDegrees(-60));
        tagRotationsMap.put(10, Rotation2d.fromDegrees(0));
        tagRotationsMap.put(11, Rotation2d.fromDegrees(60));

        // TODO: Should these be flipped?
        tagRotationsMap.put(17, Rotation2d.fromDegrees(60));
        tagRotationsMap.put(18, Rotation2d.fromDegrees(0));
        tagRotationsMap.put(19, Rotation2d.fromDegrees(-60));
        tagRotationsMap.put(20, Rotation2d.fromDegrees(-120));
        tagRotationsMap.put(21, Rotation2d.fromDegrees(180));
        tagRotationsMap.put(22, Rotation2d.fromDegrees(120));
    }
    public static final double TARGET_DEBOUNCE_TIME = 0.2;

    /* INSTANCE VARIABLES */
    private int tagCount;
    private int[] validIDs = {}; // TODO: set these
    public String cameraName;
    private double tx;
    private double ty;
    private Debouncer targetDebouncer = new Debouncer(TARGET_DEBOUNCE_TIME, DebounceType.kFalling);

    public static final double angleVelocityTolerance = 360 * Math.PI / 180; // in radians per sec

    private double cameraHeightMeters;
    public double cameraAngle;
    public double cameraOffsetX; // right is positive
    public double cameraOffsetY; //forward is positive
    private double angleMult;
    
    private DoublePublisher yDistPub;
    private DoublePublisher xDistPub;
    private DoublePublisher horizontalDistPub;

    // TODO setup camera IPs?
    // https://docs.limelightvision.io/docs/docs-limelight/getting-started/FRC/best-practices
    public Limelight(String cameraName, double cameraHeightMeters, double cameraAngle, double cameraOffsetX, double cameraOffsetY, boolean cameraUpsideDown) {
        this.cameraName = cameraName;
        this.cameraHeightMeters = cameraHeightMeters;
        this.cameraAngle = cameraAngle;
        this.cameraOffsetX = cameraOffsetX;
        this.cameraOffsetY = cameraOffsetY;
        LimelightHelpers.SetFiducialIDFiltersOverride(cameraName, validIDs);
        if (cameraUpsideDown) {
            angleMult = -1;
        } else {
            angleMult = 1;
        }

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable lightTable = inst.getTable(cameraName);
        
        yDistPub = lightTable.getDoubleTopic("Y Distance").publish();
        xDistPub = lightTable.getDoubleTopic("X Distance").publish();
        horizontalDistPub = lightTable.getDoubleTopic("Horizontal Distance").publish();
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

    public void disable() {
        // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2#using-limelight-4s-built-in-imu-with-imumode_set--setimumode
        // https://docs.limelightvision.io/docs/docs-limelight/software-change-log#limelight-os-20251-final-release---22425-test-release---21825
        LimelightHelpers.SetIMUMode(cameraName, 1); // If not moving reset internal IMU       
        // LimelightHelpers.setLimelightNTDouble(cameraName, "throttle_set", 200); // manage thermals
    }

    public void setGyroMode(int mode) {
        LimelightHelpers.SetIMUMode(cameraName, mode);
    }

    public void enable() {
        LimelightHelpers.SetIMUMode(cameraName, 4); // if moving use builtin, maybe change to 4
        // LimelightHelpers.setLimelightNTDouble(cameraName, "throttle_set", 0); //TODO check needs to be 1? // manage thermals
    }

    public RawFiducial getClosestTag() {
        RawFiducial[] tags = LimelightHelpers.getRawFiducials(cameraName);
        if (tags.length == 0) {
            System.out.println("TAGS IS ZERO");
            return null;
        }
        RawFiducial largest = tags[0];
        for (RawFiducial tag : tags) {
            if (tag.ta > largest.ta) {
                largest = tag;
            }
        }
        return largest;
    }

    public Rotation2d getClosestTagAngle() {
        int closestId = getClosestTag().id;
        return tagRotationsMap.get(closestId);
    }

    public void poseEstimationMegatag2() {


        // System.out.println(RobotContainer.drivetrain.getWrappedHeading().getDegrees());
        double angle = (RobotContainer.drivetrain.getWrappedHeading().getDegrees() + 360) % 360;
        LimelightHelpers.SetRobotOrientation(cameraName, angle, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
        // System.out.println(Utils.getCurrentTimeSeconds());
        boolean shouldRejectUpdate = false;
        if (mt2 != null) {
            RawFiducial[] tags = mt2.rawFiducials;
            int[] ids = new int[tags.length];
            for (int i = 0; i < tags.length; i++) {
                ids[i] = tags[i].id;
            }
            Logger.recordOutput(cameraName + "/SeenTags", ids); 
            if (mt2.tagCount == 0) {
                //rejects current measurement if there are no aprilTags
                shouldRejectUpdate = true;
            }
            if (Math.abs(RobotContainer.drivetrain.getCurrentSpeeds().omegaRadiansPerSecond) > angleVelocityTolerance) {
                shouldRejectUpdate = true;
            }
            if (mt2.pose.getTranslation().getDistance(RobotContainer.drivetrain.getRobotPose().getTranslation()) > 0.3 && !DriverStation.isDisabled() && !DriverStation.isTeleopEnabled()) {
                shouldRejectUpdate = true;
            }
            if (Math.abs(PoseUtils.wrapRotation(mt2.pose.getRotation()).minus(PoseUtils.wrapRotation(RobotContainer.drivetrain.getRobotPose().getRotation())).getDegrees()) > 3) {
                shouldRejectUpdate = true;
            }
            if (mt2.avgTagDist > 4) {
                shouldRejectUpdate = true;
            } 
            //adds vision measurement if conditions are met
            if (!shouldRejectUpdate) {
                Logger.recordOutput(cameraName + "/mt2Pose", mt2.pose);
                Logger.recordOutput(cameraName + "/Calculated stdevs", Math.pow(0.5, mt2.tagCount) * 2 * mt2.avgTagDist);
                // Vector<N3> = VecBuilder.fill
                // RobotContainer.drivetrain.addVisionMeasurement(
                //     mt2.pose,
                //     Utils.fpgaToCurrentTime(mt2.timestampSeconds),
                //     // VecBuilder.fill(0.000716, 0.0003, Double.POSITIVE_INFINITY));
                //     VecBuilder.fill(Math.pow(0.5, mt2.tagCount) * 2 * mt2.avgTagDist, Math.pow(0.5, mt2.tagCount) * 2 * mt2.avgTagDist, Double.POSITIVE_INFINITY)
                // );
            } else {
                Logger.recordOutput(cameraName + "/mt2PoseRejected", mt2.pose);
            }
        }
    }

    //TODO: Do we need these / check if the trig is right
    

    public double getDistanceToTag(double tagHeightMeters) {
        if (hasValidTarget()) {
            double distance = getStraightDistanceToTag(tagHeightMeters) - cameraOffsetY;
            return distance / Math.cos((Math.PI / 180.0) * getTX());
        }
        return 0;
    }

    public double getStraightDistanceToTag(double tagHeightMeters) {
        if (hasValidTarget()) {
            double distance = (tagHeightMeters - cameraHeightMeters)
                    / Math.tan(
                            (Math.PI / 180.0)
                                    * (cameraAngle + getTY()));
            return distance + cameraOffsetY;
        }
        return 0;
    }

    public double getHorizontalDistanceToTag(double tagHeightMeters) {
        if (hasValidTarget()) {
            double distance = getStraightDistanceToTag(tagHeightMeters) - cameraOffsetY;
            
            distance = distance * Math.tan(getTX() * (Math.PI / 180.0));
            return distance + cameraOffsetX;
        }
        return 0;
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

    @AutoLogOutput
    public double getTX() {
        if (Robot.isReal()) {
            return tx * angleMult;
        } else {
            return tx;
        }
    }

    @AutoLogOutput
    public double getTY() {
        if (Robot.isReal()) {
            return ty * -angleMult;
        } else {
            return ty;
        }
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

    private Translation2d tagPosition = LimelightSim.loadDefaultFieldLayout().getTagPose(18).get().toPose2d().getTranslation();
    
    private long lastHeartBeat = 0;
    private long frameCount = 0;
    private long aprilTagFrameCount = 0;
    private double startTime = -1;
    public void periodic() {
        // tagID = (int) Limetable.getEntry("tid").getDouble(-1);
        // TODO if you get a pose estimate in the frame before this is applied it may not work
        tx = LimelightHelpers.getTX(cameraName);
        ty = LimelightHelpers.getTY(cameraName);
        RawFiducial[] allTags = LimelightHelpers.getRawFiducials(cameraName);
        int numValidTags = 0;
        for(LimelightHelpers.RawFiducial t : allTags) {
            if(t.distToCamera < 4.0) {
                numValidTags++;
            }
        }

        int[] validTags = new int[numValidTags];
        int counter = 0;
        for(RawFiducial t : allTags) {
            if(t.distToCamera < 4.0) {
                validTags[counter] = t.id;
                counter++;
            }
        }
     //   LimelightHelpers.SetFiducialIDFiltersOverride(cameraName, validTags);
        // poseEstimationMegatag2();
        xDistPub.set(getHorizontalDistanceToReef());
        yDistPub.set(getStraightDistanceToReef());
        horizontalDistPub.set(getDistanceToReef());
        
        Logger.recordOutput(cameraName + "/Tag18/tagID", getTagID());
        if (getTagID() == 18) {
            Logger.recordOutput(cameraName + "/Tag18/tx", tx);
            Logger.recordOutput(cameraName + "/Tag18/ty", ty);
        }

        if (startTime < 0) {
            startTime = Timer.getFPGATimestamp();
        }
        var entry = LimelightHelpers.getLimelightNTTableEntry(cameraName, "hb");
        long heartBeat = entry.getInteger(0);
        if (heartBeat != lastHeartBeat) {
            lastHeartBeat = heartBeat;
            frameCount++;
            Logger.recordOutput(cameraName + "/frameCount", frameCount);
            Logger.recordOutput(cameraName + "/framesPerSecond", frameCount / (Timer.getFPGATimestamp() - startTime));
            if (LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName) != null) {
                aprilTagFrameCount++;
                Logger.recordOutput(cameraName + "/aprilTagFrameCount", aprilTagFrameCount);
                Logger.recordOutput(cameraName + "/aprilTagFramesPerSecond", aprilTagFrameCount / (Timer.getFPGATimestamp() - startTime));
            }
        }

        double[] corners = getCorners(18);
        if (corners != null) {
            // Get min and max because limelight documentation says corners can be in any order?
            // Maybe we should determine the bottom left and bottom right corners and use those?
            // Not sure it makes a difference
            double minX = Double.MAX_VALUE;
            double maxX = Double.MIN_VALUE;
            double minY = Double.MAX_VALUE;
            double maxY = Double.MIN_VALUE;
            for (int i=0; i<corners.length; i+=2) {
                if (corners[i] < minX) {
                    minX = corners[i];
                }
                if (corners[i] > maxX) {
                    maxX = corners[i];
                }
                if (corners[i+1] < minY) {
                    minY = corners[i+1];
                }
                if (corners[i+1] > maxY) {
                    maxY = corners[i+1];
                }
            }
            // Width and height of tag in pixels
            double width = maxX - minX;
            double height = maxY - minY;
            Logger.recordOutput(cameraName + "/Tag/width", width);
            Logger.recordOutput(cameraName + "/Tag/height", height);
            // Width and height of tag as angles (radians)
            double HORIZONTAL_FOCAL_LENGTH = (1280.0 / 2.0) / Math.tan(Units.degreesToRadians(82.0 / 2.0));
            double VERTICAL_FOCAL_LENGTH = (720.0 / 2.0) / Math.tan(Units.degreesToRadians(52.8 / 2.0));
            double xAngle = Math.atan(width / HORIZONTAL_FOCAL_LENGTH);
            double yAngle = Math.atan(height / VERTICAL_FOCAL_LENGTH);
            Logger.recordOutput(cameraName + "/Tag/xAngle", Units.radiansToDegrees(xAngle));
            Logger.recordOutput(cameraName + "/Tag/yAngle", Units.radiansToDegrees(yAngle));

            // Size of tags are 8.75 inches
            // So x distance to tag = size / tan()
            double x = Math.abs(Units.inchesToMeters(8.75) / Math.tan(yAngle));
            // y distance to tag is x distance * tan(x angle to tag)
            double y = x * Math.tan(Units.degreesToRadians(getTX()));
            Logger.recordOutput(cameraName + "/Tag/x", x);
            Logger.recordOutput(cameraName + "/Tag/y", y);
            Logger.recordOutput(cameraName + "/Tag/dist", Math.sqrt(x*x + y*y));

            Translation2d robotPosition = RobotContainer.drivetrain.getRobotPose().getTranslation();
            double x2 = Math.abs(robotPosition.getX() - tagPosition.getX());
            double y2 = Math.abs(robotPosition.getY() - tagPosition.getY());
            Logger.recordOutput(cameraName + "/Tag/angle2", RobotContainer.drivetrain.getRobotPose().getRotation().getDegrees());
            Logger.recordOutput(cameraName + "/Tag/x2", x2);
            Logger.recordOutput(cameraName + "/Tag/y2", y2);
            Logger.recordOutput(cameraName + "/Tag/dist2", Math.sqrt(x2*x2 + y2*y2));

            double yAngle2 = Math.atan(Units.inchesToMeters(8.75) / x2);
            Logger.recordOutput(cameraName + "/Tag/yAngle2", yAngle2);

            double verticalFocalLength2 = height / Math.tan(yAngle2);
            Logger.recordOutput(cameraName + "/Tag/verticalFocalLength", VERTICAL_FOCAL_LENGTH);
            Logger.recordOutput(cameraName + "/Tag/verticalFocalLength2", verticalFocalLength2);
        }
    }

    public Command ifHasTarget(Command cmd) {
        return cmd.onlyWhile(this::hasValidTarget);
    }

    public double getCameraHeightMeters() {
        return cameraHeightMeters;
    }
    public double getCameraOffsetX() {
        return cameraOffsetX;
    }
    public double getCameraOffsetY() {
        return cameraOffsetY;
    }
    public double getCameraAngle() {
        return cameraAngle;
    }
    public void setPriorityTagID(int id) {
        LimelightHelpers.setPriorityTagID(cameraName, id);
    }

    public int getLargestAprilTag(Set<Integer> validIDs) {
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(cameraName);
        int largestID = -1;
        double largestArea = 0;
        for (RawFiducial fiducial : fiducials) {
            if (validIDs.contains(fiducial.id) && fiducial.ta > largestArea) {
                largestArea = fiducial.ta;
                largestID = fiducial.id;
            }
        }
        return largestID;
    }


    public double[] getCorners(int tagId) {

        var entry = LimelightHelpers.getLimelightNTTableEntry(cameraName, "tcornxy");
        var tcornxy = entry.getDoubleArray(new double[0]);

        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(cameraName);
        for (int i=0; i<fiducials.length; i++) {
            if (fiducials[i].id == tagId) {
                if (tcornxy.length < i*8+8) {
                    return null;
                }
                return new double[] { tcornxy[i*8], tcornxy[i*8+1], tcornxy[i*8+2], tcornxy[i*8+3], tcornxy[i*8+4], tcornxy[i*8+5], tcornxy[i*8+6], tcornxy[i*8+7] };
            }
        }
        return null;
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
