package frc.robot.vision;

import java.util.HashMap;
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
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.PoseUtils;
import frc.robot.vision.LimelightHelpers.RawFiducial;

public class Limelight extends SubsystemBase {

    /* CONSTANTS */
    public static final double coralStationTagHeightMeters = 1.35255; // make sure these two are correct
    // does it need to be to the center of the tag?
    public static final double reefTagHeightMeters = // 0.174625;
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
    public double cameraOffsetY; // forward is positive
    private double angleMult;

    private boolean hasTipped;

    private DoublePublisher yDistPub;
    private DoublePublisher xDistPub;
    private DoublePublisher horizontalDistPub;

    // TODO setup camera IPs?
    // https://docs.limelightvision.io/docs/docs-limelight/getting-started/FRC/best-practices
    public Limelight(String cameraName, double cameraHeightMeters, double cameraAngle, double cameraOffsetX,
            double cameraOffsetY, boolean cameraUpsideDown) {
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
        // LimelightHelpers.setLimelightNTDouble(cameraName, "throttle_set", 200); //
        // manage thermals
    }

    public void setGyroMode(int mode) {
        LimelightHelpers.SetIMUMode(cameraName, mode);
    }

    public void enable() {
        LimelightHelpers.SetIMUMode(cameraName, 1); // if moving use builtin, maybe change to 4
        // LimelightHelpers.setLimelightNTDouble(cameraName, "throttle_set", 0); //TODO
        // check needs to be 1? // manage thermals
    }

    public RawFiducial getClosestTag() {
        RawFiducial[] tags = LimelightHelpers.getRawFiducials(cameraName);
        if (tags.length == 0) {
            return null;
        }
        RawFiducial largest = tags[0];
        for (RawFiducial tag : tags) {
            if (tag.distToRobot > largest.distToRobot) {
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

        double angle = (RobotContainer.drivetrain.getWrappedHeading().getDegrees() + 360) % 360;
        LimelightHelpers.SetRobotOrientation(cameraName, angle, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

        boolean overrideReject = false;
        boolean isTipping = Math.abs(RobotContainer.drivetrain.getPigeon2().getPitch().getValueAsDouble()) > 2
                || Math.abs(RobotContainer.drivetrain.getPigeon2().getRoll().getValueAsDouble()) > 2;
        Logger.recordOutput(cameraName + "/isTipping", isTipping);

        if (hasTipped && !isTipping) {
            overrideReject = true;
        }

        boolean shouldRejectUpdate = false;
        int rejectReason = 0;
        if (mt2 != null) {
            Optional<Pose2d> optPastRobotPose = RobotContainer.drivetrain.getPoseAtTime(mt2.timestampSeconds);
            if (optPastRobotPose.isPresent()) {
                Logger.recordOutput(cameraName + "/PastRobotPose", optPastRobotPose.get());
            }
            Pose2d pastRobotPose = RobotContainer.drivetrain.getRobotPose();
            // Pose2d pastRobotPose = optPastRobotPose.orElseGet(() -> RobotContainer.drivetrain.getRobotPose());
            Logger.recordOutput(cameraName + "/timestampSeconds", mt2.timestampSeconds);
            RawFiducial[] tags = mt2.rawFiducials;
            int[] ids = new int[tags.length];
            for (int i = 0; i < tags.length; i++) {
                ids[i] = tags[i].id;
            }
            Logger.recordOutput(cameraName + "/SeenTags", ids);
            Logger.recordOutput(cameraName + "/PoseLatency", mt2.timestampSeconds - Timer.getFPGATimestamp());
            if (mt2.tagCount == 0) {
                // rejects current measurement if there are no aprilTags
                shouldRejectUpdate = true;
                rejectReason = 1;
            }
            if (Math.abs(RobotContainer.drivetrain.getCurrentSpeeds().omegaRadiansPerSecond) > angleVelocityTolerance) {
                shouldRejectUpdate = true;
                rejectReason = 2;
            }
            if ((mt2.pose.getTranslation().getDistance(pastRobotPose.getTranslation()) > 0.9
                    && !DriverStation.isDisabled() && !DriverStation.isTeleopEnabled())) {
                shouldRejectUpdate = true;
                rejectReason = 3;
            }
            if (Math.abs(PoseUtils.wrapRotation(mt2.pose.getRotation())
                    .minus(PoseUtils.wrapRotation(pastRobotPose.getRotation())).getDegrees()) > 3) {
                shouldRejectUpdate = true;
                rejectReason = 4;
            }
            if (mt2.avgTagDist > 4) {
                shouldRejectUpdate = true;
                rejectReason = 5;
            }
            // if (isTipping) {
            // shouldRejectUpdate = true;
            // }
            // adds vision measurement if conditions are met
            if (!shouldRejectUpdate) {
                Logger.recordOutput(cameraName + "/mt2Pose", mt2.pose);
                Logger.recordOutput(cameraName + "/Calculated stdevs",
                        Math.pow(0.5, mt2.tagCount) * 2 * mt2.avgTagDist);
                // Vector<N3> = VecBuilder.fill
                RobotContainer.drivetrain.addVisionMeasurement(
                        mt2.pose,
                        Utils.fpgaToCurrentTime(mt2.timestampSeconds),
                        // VecBuilder.fill(0.000716, 0.0003, Double.POSITIVE_INFINITY));
                        VecBuilder.fill(Math.pow(0.5, mt2.tagCount) * 2 * mt2.avgTagDist,
                                Math.pow(0.5, mt2.tagCount) * 2 * mt2.avgTagDist, Double.POSITIVE_INFINITY));
            } else {
                Logger.recordOutput(cameraName + "/mt2PoseRejected", mt2.pose);
                Logger.recordOutput(cameraName + "/rejectReason", rejectReason);
            }
        }
    }

    // TODO: Do we need these / check if the trig is right

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

    // TODO: Do we need these / check if the trig is right
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

    // TODO: Do we need these / check if the trig is right
    // public double getStraightDistanceToTag() {
    // if (hasValidTarget())
    // return goalHeightReef / (Math.tan(Math.toRadians(getTY() +
    // limelightOffsetAngleVertical)));
    // return 0;
    // }

    // TODO: Do we need these / check if the trig is right
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

        if (Math.abs(RobotContainer.drivetrain.getPigeon2().getPitch().getValueAsDouble()) > 0.3
                || Math.abs(RobotContainer.drivetrain.getPigeon2().getRoll().getValueAsDouble()) > 0.3) {
            hasTipped = true;
        }
        // tagID = (int) Limetable.getEntry("tid").getDouble(-1);
        // TODO if you get a pose estimate in the frame before this is applied it may
        // not work
        tx = LimelightHelpers.getTX(cameraName);
        ty = LimelightHelpers.getTY(cameraName);
        RawFiducial[] allTags = LimelightHelpers.getRawFiducials(cameraName);
        int numValidTags = 0;
        for (LimelightHelpers.RawFiducial t : allTags) {
            if (t.distToCamera < 4.0) {
                numValidTags++;
            }
        }

        int[] validTags = new int[numValidTags];
        int counter = 0;
        for (RawFiducial t : allTags) {
            if (t.distToCamera < 4.0) {
                validTags[counter] = t.id;
                counter++;
            }
        }
        // LimelightHelpers.SetFiducialIDFiltersOverride(cameraName, validTags);
        poseEstimationMegatag2();
        xDistPub.set(getHorizontalDistanceToReef());
        yDistPub.set(getStraightDistanceToReef());
        horizontalDistPub.set(getDistanceToReef());

        double[] poseArr = LimelightHelpers.getBotPose_TargetSpace(cameraName);
        Pose2d botPose = new Pose2d();
        if (poseArr.length >= 6) {
            botPose = new Pose2d(poseArr[0], poseArr[2], Rotation2d.fromDegrees(poseArr[4]));
        }
        Logger.recordOutput(cameraName + "/IMUYaw",
                LimelightHelpers.getIMUData(cameraName).robotYaw * (Math.PI / 180.0)); // TODO should be yaw?
        Logger.recordOutput(cameraName + "/BotPoseTargetSpace", botPose);
        Logger.recordOutput(cameraName + "/BotPose3dTargetSpace",
                LimelightHelpers.getBotPose3d_TargetSpace(cameraName));

        var entry = LimelightHelpers.getLimelightNTTableEntry(cameraName, "tcornxy");
        if (entry != null) {
            var tcornxy = entry.getDoubleArray(new double[0]);
            if (tcornxy != null && tcornxy.length > 0) {
                Logger.recordOutput(cameraName + "/tcornxy", tcornxy);
            }
        }
    }

    public Command flashLEDs() {
        return Commands.sequence(
            Commands.runOnce(() -> LimelightHelpers.setLEDMode_ForceBlink(cameraName)),
            Commands.waitSeconds(0.6),
            Commands.runOnce(() -> LimelightHelpers.setLEDMode_ForceOff(cameraName))
        );
    }



    public Command ifHasTarget(Command cmd) {
        return cmd.onlyWhile(this::hasValidTarget);
    }
}
