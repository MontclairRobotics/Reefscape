package frc.robot.vision;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.PoseUtils;

public class Limelight extends SubsystemBase {

    // Last time we accepted a pose from the vision system
    public static double lastAcceptablePoseTime = -1;

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
    private double cameraOffsetX; // right is positive
    private double cameraOffsetY; //forward is positive
    private double angleMult;
    
    // Buffer of past robot poses
    private TimeInterpolatableBuffer<Pose2d> robotPoseBuffer;

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

        robotPoseBuffer = TimeInterpolatableBuffer.createBuffer(2);

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

    public void poseEstimationMegatag2() {


        // System.out.println(RobotContainer.drivetrain.getWrappedHeading().getDegrees());
        double angle = (RobotContainer.drivetrain.getWrappedHeading().getDegrees() + 360) % 360;
        LimelightHelpers.SetRobotOrientation(cameraName, angle, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
        // System.out.println(Utils.getCurrentTimeSeconds());

        boolean shouldRejectUpdate = false;
        if (mt2 != null) { 
            // Reject if there are no aprilTags
            if (mt2.tagCount == 0) {
                shouldRejectUpdate = true;
            }
            // Reject if robot is spinning too fast
            if (Math.abs(RobotContainer.drivetrain.getCurrentSpeeds().omegaRadiansPerSecond) > angleVelocityTolerance) {
                shouldRejectUpdate = true;
            }
            // Reject if tag distance is too far
            if (mt2.avgTagDist > 4) {
                shouldRejectUpdate = true;
            }
            // Get pose of robot at time of vision measurement
            // If we don't have a pose at the time of the vision measurement, just use current pose
            // Using current pose is better than nothing
            // It's more likely to get rejected if robot is moving, but if robot isn't moving much, it'll get accepted
            // And that's fine
            Optional<Pose2d> optPastRobotPose = robotPoseBuffer.getSample(mt2.timestampSeconds);
            Pose2d robotPose = optPastRobotPose.isPresent() ? optPastRobotPose.get() : RobotContainer.drivetrain.getRobotPose();

            // Reject if pose is different of angle from robot pose
            // This is fixed because even if robot slips or is bump or anything, pigeon should still be correct
            // So if pose is off from that, pose is probably wrong
            double acceptableAngle = 3; 
            if (Math.abs(PoseUtils.wrapRotation(mt2.pose.getRotation()).minus(PoseUtils.wrapRotation(robotPose.getRotation())).getDegrees()) > acceptableAngle) {
                shouldRejectUpdate = true;
            }

            // Reject if pose is too far from robot pose
            // Accepted difference of vision to robot pose grows with time
            // 20cm to start, then 10cm per second since vision estimate was accepted
            // We assume that if 
            double accepatbleDistance = 0.2 + 0.1 * (Timer.getFPGATimestamp() - lastAcceptablePoseTime);
            if (mt2.pose.getTranslation().getDistance(robotPose.getTranslation()) > accepatbleDistance && !DriverStation.isDisabled()) {
                shouldRejectUpdate = true;
            } else {
                // Last acceptable pose time is update, when we have a vision measurement that
                // we are rejecting only because of distance
                // We don't want accepatbleDistance to grov if we aren't getting poses at all
                // Or if the poses we are getting are just bad
                // Only if we are getting good poses that are just a little off
                if (!shouldRejectUpdate) {
                    lastAcceptablePoseTime = Timer.getFPGATimestamp();
                }
            }


            //adds vision measurement if conditions are met
            if (!shouldRejectUpdate) {
                Logger.recordOutput(cameraName + "/mt2Pose", mt2.pose);
                Logger.recordOutput(cameraName + "/Calculated stdevs", Math.pow(0.5, mt2.tagCount) * 2 * mt2.avgTagDist);
                // Vector<N3> = VecBuilder.fill
                RobotContainer.drivetrain.addVisionMeasurement(
                    mt2.pose,
                    Utils.fpgaToCurrentTime(mt2.timestampSeconds),
                    // VecBuilder.fill(0.000716, 0.0003, Double.POSITIVE_INFINITY));
                    VecBuilder.fill(Math.pow(0.5, mt2.tagCount) * 2 * mt2.avgTagDist, Math.pow(0.5, mt2.tagCount) * 2 * mt2.avgTagDist, Double.POSITIVE_INFINITY)
                );
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
        return tx * angleMult;
    }

    @AutoLogOutput
    public double getTY() {
        return ty * -angleMult;
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
        // Update the robot pose buffer
        Pose2d robotPose = RobotContainer.drivetrain.getRobotPose();
        robotPoseBuffer.addSample(Timer.getFPGATimestamp(), robotPose);

        // On first run, set last accepted pose time to current time
        if (lastAcceptablePoseTime < 0) {
            lastAcceptablePoseTime = Timer.getFPGATimestamp();
        }

        // tagID = (int) Limetable.getEntry("tid").getDouble(-1);
        tx = LimelightHelpers.getTX(cameraName);
        ty = LimelightHelpers.getTY(cameraName);
        poseEstimationMegatag2();
        xDistPub.set(getHorizontalDistanceToReef());
        yDistPub.set(getStraightDistanceToReef());
        horizontalDistPub.set(getDistanceToReef());
    }

    public Command ifHasTarget(Command cmd) {
        return cmd.onlyWhile(this::hasValidTarget);
    }
}
