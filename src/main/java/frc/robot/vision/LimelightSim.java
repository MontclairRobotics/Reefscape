package frc.robot.vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PnpResult;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSim extends SubsystemBase {

    public static enum LimelightModel {
        LIMELIGHT_1(54.0),
        LIMELIGHT_2(59.6),
        LIMELIGHT_3(80),
        LIMELIGHT_3G(80),
        LIMELIGHT_4(80);

        public double fov;

        private LimelightModel(double fov) {
            this.fov = fov;
        }
    }
    public static enum LimelightResolution {

        RESOLUTION_320x200(320, 200),
        RESOLUTION_720x480(720, 480);

        public int x;
        public int y;

        private LimelightResolution(int x, int y) {
            this.x=x;
            this.y=y;
        }
    }

    // Vision simulation
    private VisionSystemSim visionSim;
    // Robot pose
    private Supplier<Pose2d> robotPoseSupplier;
    // Cameras
    private ArrayList<LimelightCameraSim> cameras;
    // Field layout of april tags
    private AprilTagFieldLayout fieldLayout;

    // Buffer of past robot poses
    private TimeInterpolatableBuffer<Pose2d> robotPoseBuffer;


    // ------------------------------------------------------------------------------------------
    // CONSTRUCTORS
    /**
     * Constructor which uses default field layout
     */
    public LimelightSim(Supplier<Pose2d> robotPoseSupplier) {
        this(robotPoseSupplier, loadDefaultFieldLayout());
    }
    /**
     * Constructor
     */
    public LimelightSim(Supplier<Pose2d> robotPoseSupplier, AprilTagFieldLayout fieldLayout) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.fieldLayout = fieldLayout;
        this.cameras = new ArrayList<LimelightCameraSim>();

        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(fieldLayout);

        robotPoseBuffer = TimeInterpolatableBuffer.createBuffer(2);
    }



    // ------------------------------------------------------------------------------------------
    // METHODS
    /**
     * Add a new camera
     * @param cameraName
     * @param model
     * @param resolutionX
     * @param resolutionY
     */
    public void addCamera(Transform3d robotToCamera, String cameraName, LimelightModel model, LimelightResolution resolution, double fps, double latency) {
        LimelightCameraSim cameraSim = new LimelightCameraSim(this, robotToCamera, cameraName, model, resolution, fps, latency);
        visionSim.addCamera(cameraSim.cameraSim, robotToCamera);
        cameras.add(cameraSim);
    }


    /**
     * Periodic update. Ensure this is called repeatedly
     * Updates all cameras
     */
    @Override
    public void periodic() {
        Pose2d robotPose = robotPoseSupplier.get();
        
        robotPoseBuffer.addSample(Timer.getFPGATimestamp(), robotPose);
        visionSim.update(robotPose);

        for (LimelightCameraSim camera : cameras) {
            camera.periodic();
        }
    }



    // ------------------------------------------------------------------------------------------
    // STATIC METHODS
    /**
     * Load the field layout from the default field
     */
    private static AprilTagFieldLayout loadDefaultFieldLayout() {
        try {
            return AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
    }
    /**
     * Flip a field position to the other side of the field, maintaining a blue alliance origin
     *
     * @param pos The position to flip
     * @return The flipped position
     */
    public static Translation3d flipFieldPosition(Translation3d pos) {
        return switch (FlippingUtil.symmetryType) {
            case kMirrored -> new Translation3d(FlippingUtil.fieldSizeX - pos.getX(), pos.getY(), pos.getZ());
            case kRotational -> new Translation3d(FlippingUtil.fieldSizeX - pos.getX(), FlippingUtil.fieldSizeY - pos.getY(), pos.getZ());
        };
    }

    /**
     * Flip a field rotation to the other side of the field, maintaining a blue alliance origin
     *
     * @param rotation The rotation to flip
     * @return The flipped rotation
     */
    public static Rotation3d flipFieldRotation(Rotation3d rotation) {
        // TODO: Is this correct????
        return switch (FlippingUtil.symmetryType) {
            case kMirrored -> new Rotation3d(rotation.getX(), rotation.getY(), Rotation2d.kPi.getRadians() - rotation.getZ());
            case kRotational -> new Rotation3d(rotation.getX(), rotation.getY(), rotation.getZ() - Rotation2d.kPi.getRadians());
        };
    }

    /**
     * Flip a field pose to the other side of the field, maintaining a blue alliance origin
     *
     * @param pose The pose to flip
     * @return The flipped pose
     */
    public static Pose3d flipFieldPose(Pose3d pose) {
        return new Pose3d(
            flipFieldPosition(pose.getTranslation()), flipFieldRotation(pose.getRotation()));
    }

    public static double getDistance(Pose3d pose1, Pose2d pose2) {
        System.out.println("getDistnce: " + pose1.getX() + " " + pose2.getX() + " " + pose1.getY() + " " + pose2.getY());
        double result = Math.sqrt(Math.pow(pose1.getX() - pose2.getX(), 2) + Math.pow(pose1.getY() - pose2.getY(), 2));
        System.out.println("getDistnce: " + result);
        return result;
        }




    // ------------------------------------------------------------------------------------------
    // SUB CLASS
    /**
     * Information on all cameras
     */
    private static class LimelightCameraSim {
        public LimelightSim limelightSim;

        public String name;
        public PhotonCamera camera;
        public PhotonCameraSim cameraSim;
        public SimCameraProperties cameraProperties;
        public Transform3d robotToCamera;

        // Frame counter
        private int frameCounter = 0;

        // Network tables
        private IntegerPublisher tv;
        private DoublePublisher tx;
        private DoublePublisher ty;
        private DoublePublisher txnc;
        private DoublePublisher tync;
        private DoublePublisher ta;
        private DoublePublisher tl;
        private DoublePublisher cl;
        private DoubleArrayPublisher t2d;
        private IntegerPublisher getpipe;
        private StringPublisher getpipetype;
        private DoublePublisher hb;
        private DoubleArrayPublisher hw;

        private DoubleArrayPublisher botpose;
        private DoubleArrayPublisher botpose_wpiblue;
        private DoubleArrayPublisher botpose_wpired;
        private DoubleArrayPublisher botpose_orb;
        private DoubleArrayPublisher botpose_orb_wpiblue;
        private DoubleArrayPublisher botpose_orb_wpired;
        private DoubleArrayPublisher camerapose_targetspace;
        private DoubleArrayPublisher targetpose_cameraspace;
        private DoubleArrayPublisher targetpose_robotspace;
        private DoubleArrayPublisher botpose_targetspace;
        private DoubleArrayPublisher camerapose_robotspace;
        private IntegerPublisher tid;
        private DoubleArrayPublisher stddevs;

        private StructPublisher<Pose2d> odometryPosePublisher;

        private DoubleArraySubscriber camerapose_robotspace_set;
        private IntegerSubscriber priorityid;
        private DoubleArraySubscriber robot_orientation_set;
        private DoubleArraySubscriber fiducial_id_filters_set;
        private DoubleArraySubscriber fiducial_offset_set;

        public LimelightCameraSim(LimelightSim limelightSim, Transform3d robotToCamera, String name, SimCameraProperties cameraProperties) {
            this.limelightSim = limelightSim;
            this.name = name;
            this.cameraProperties = cameraProperties;
            this.robotToCamera = robotToCamera;

            // Represent the camera used in code (we read values from this)
            camera = new PhotonCamera(name);
            // The simulation of this camera. Its values used in real robot code will be updated.
            cameraSim = new PhotonCameraSim(camera, cameraProperties);

            setupNetworkTables();
        }

        public LimelightCameraSim(LimelightSim limelightSim, Transform3d robotToCamera, String cameraName, LimelightModel model, LimelightResolution resolution, double fps, double latency) {
            this(limelightSim, robotToCamera, cameraName, createSimCameraProperties(model, resolution, fps, latency));
        }

        private static SimCameraProperties createSimCameraProperties(LimelightModel model, LimelightResolution resolution, double fps, double latency) {
            SimCameraProperties cameraProp = new SimCameraProperties();

            // A 640 x 480 camera with a 100 degree diagonal FOV.
            cameraProp.setCalibration(resolution.x, resolution.y, Rotation2d.fromDegrees(model.fov));
            // Approximate detection noise with average and standard deviation error in pixels.
            cameraProp.setCalibError(0.25, 0.08);
            // Set the camera image capture framerate (Note: this is limited by robot loop rate).
            cameraProp.setFPS(fps);
            // The average and standard deviation in milliseconds of image data latency.
            // We set std dev to 15%
            cameraProp.setAvgLatencyMs(latency);
            cameraProp.setLatencyStdDevMs(latency * 0.15);

            return cameraProp;
        }

        /**
         * Sets up all network table publishers and subscribers
         */
        private void setupNetworkTables() {
            NetworkTableInstance inst = NetworkTableInstance.getDefault();
            NetworkTable limelightTable = inst.getTable(name);
            tv = limelightTable.getIntegerTopic("tv").publish();
            tx = limelightTable.getDoubleTopic("tx").publish();
            ty = limelightTable.getDoubleTopic("ty").publish();
            txnc = limelightTable.getDoubleTopic("txnc").publish();
            tync = limelightTable.getDoubleTopic("tync").publish();
            ta = limelightTable.getDoubleTopic("ta").publish();
            tl = limelightTable.getDoubleTopic("tl").publish();
            cl = limelightTable.getDoubleTopic("cl").publish();
            t2d = limelightTable.getDoubleArrayTopic("t2d").publish();
            getpipe = limelightTable.getIntegerTopic("getpipe").publish();
            getpipetype = limelightTable.getStringTopic("getpipetype").publish();
            hb = limelightTable.getDoubleTopic("hb").publish();
            hw = limelightTable.getDoubleArrayTopic("hw").publish();

            botpose = limelightTable.getDoubleArrayTopic("botpose").publish();
            botpose_wpiblue = limelightTable.getDoubleArrayTopic("botpose_wpiblue").publish();
            botpose_wpired = limelightTable.getDoubleArrayTopic("botpose_wpired").publish();
            botpose_orb = limelightTable.getDoubleArrayTopic("botpose_orb").publish();
            botpose_orb_wpiblue = limelightTable.getDoubleArrayTopic("botpose_orb_wpiblue").publish();
            botpose_orb_wpired = limelightTable.getDoubleArrayTopic("botpose_orb_wpired").publish();
            camerapose_targetspace = limelightTable.getDoubleArrayTopic("camerapose_targetspace").publish();
            targetpose_cameraspace = limelightTable.getDoubleArrayTopic("targetpose_cameraspace").publish();
            targetpose_robotspace = limelightTable.getDoubleArrayTopic("targetpose_robotspace").publish();
            botpose_targetspace = limelightTable.getDoubleArrayTopic("botpose_targetspace").publish();
            camerapose_robotspace = limelightTable.getDoubleArrayTopic("camerapose_robotspace").publish();
            tid = limelightTable.getIntegerTopic("tid").publish();
            stddevs = limelightTable.getDoubleArrayTopic("stddevs").publish();

            odometryPosePublisher = limelightTable.getStructTopic("Odometry Pose", Pose2d.struct).publish();

            // TODO: subscribers
            camerapose_robotspace_set = limelightTable.getDoubleArrayTopic("camerapose_robotspace").subscribe(new double[]{});
            priorityid = limelightTable.getIntegerTopic("priorityid").subscribe(0);
            robot_orientation_set = limelightTable.getDoubleArrayTopic("robot_orientation").subscribe(new double[]{});
            fiducial_id_filters_set = limelightTable.getDoubleArrayTopic("fiducial_id_filters").subscribe(new double[]{});
            fiducial_offset_set = limelightTable.getDoubleArrayTopic("fiducial_offset").subscribe(new double[]{});
        }

        /**
         * Grab data from photon simulation and publish to equivalent limelight NT entries
         */
        public void periodic() {
            List<PhotonPipelineResult> results = camera.getAllUnreadResults();
            
            if (!results.isEmpty()) {
                PhotonPipelineResult result = results.get(results.size()-1);
                frameCounter++;

                // Timestamp
                double timestamp = result.getTimestampSeconds();

                // Do we have a target?
                if (result.hasTargets()) {
                    tv.set(1);

                    PhotonTrackedTarget target = result.getBestTarget();

                    // TODO: is this correct timestamp to use?
                    double latency = Timer.getFPGATimestamp() - timestamp;
                    int tagCount = result.getTargets().size();
                    double tagAverageArea = result.getTargets().stream().mapToDouble(t -> t.getArea()).average().orElse(0.0);
                    double tagAverageDistance = result.getTargets().stream().mapToDouble(t -> t.bestCameraToTarget.getTranslation().getDistance(Translation3d.kZero)).average().orElse(0.0);

                    // System.out.println("Target: " + result.hasTargets());
                    tx.set(target.getYaw());
                    txnc.set(target.getYaw());
                    ty.set(target.getPitch());
                    tync.set(target.getPitch());
                    ta.set(target.getArea());
                    tl.set(latency/2); // Divide latency equally between camera and limelight
                    cl.set(latency/2);
                    t2d.set(new double[]{ 
                        1.0, 
                        tagCount, 
                        latency,
                        0.0,
                        target.getYaw(),
                        target.getPitch(),
                        target.getYaw(),
                        target.getPitch(),
                        target.getArea(),
                        target.getFiducialId(),
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
                    });
                    getpipe.set(0); // Dummy value
                    getpipetype.set("apriltag"); // Dummy value
                    hb.set(frameCounter);
                    hw.set(new double[]{ cameraProperties.getFPS(), 0.0, 0.0, 0.0, 0.0 }); // No values for temp and cpu

                    // Assuming the best is the main one of multi-tag
                    tid.set(target.getFiducialId());

                    Optional<Pose2d> odometryRobotPoseOption = limelightSim.robotPoseBuffer.getSample(Timer.getFPGATimestamp() - latency);
                    if (odometryRobotPoseOption.isPresent()) {
                        Pose2d odometryRobotPose = odometryRobotPoseOption.get();
                        odometryPosePublisher.set(odometryRobotPose);

                        if (result.getMultiTagResult().isPresent()) {
                            MultiTargetPNPResult multiTagResult = result.getMultiTagResult().get();
                            PnpResult pnpResult = multiTagResult.estimatedPose;
                            
                            Transform3d fieldToCamera = pnpResult.best;
                            Transform3d blueRobotTransform = fieldToCamera.plus(robotToCamera.inverse());
                            
                            Pose3d blueRobotPose = Pose3d.kZero.plus(blueRobotTransform);

                            double diff = LimelightSim.getDistance(blueRobotPose, odometryRobotPose);
                            if (diff < 0.1) {
                                publishRobotPose(blueRobotPose, latency, tagCount, tagAverageArea, tagAverageDistance);

                                stddevs.set(new double[]{0.1, 0.1, 1000.0});
                            }
                        } else {
                            Pose3d bestBlueRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), limelightSim.fieldLayout.getTagPose(target.getFiducialId()).get(), robotToCamera.inverse());
                            Pose3d altBlueRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getAlternateCameraToTarget(), limelightSim.fieldLayout.getTagPose(target.getFiducialId()).get(), robotToCamera.inverse());
                            
                            double bestDiff = LimelightSim.getDistance(bestBlueRobotPose, odometryRobotPose);
                            double altDiff = LimelightSim.getDistance(altBlueRobotPose, odometryRobotPose);

                            if (bestDiff < altDiff) {
                                if (bestDiff < 0.1) {
                                    publishRobotPose(bestBlueRobotPose, latency, tagCount, tagAverageArea, tagAverageDistance);
                                    
                                    // Higher stddevs for the non multi-tag case
                                    stddevs.set(new double[]{0.5, 0.5, 1000.0});
                                }
                            } else {
                                if (altDiff < 0.1) {
                                    publishRobotPose(altBlueRobotPose, latency, tagCount, tagAverageArea, tagAverageDistance);

                                    // Higher stddevs for the non multi-tag case
                                    stddevs.set(new double[]{0.5, 0.5, 1000.0});
                                }
                            }
                        }
                    }
                } else {
                    tv.set(0);
                }      
            }      
        }

        /**
         * Publish pose of robot to NT Tables
         */
        public void publishRobotPose(Pose3d bluePose3d, double latency, int tagCount, double tagAverageArea, double tagAverageDistance) {
            System.out.println("publishRobotPose: " + bluePose3d.getX() + " " + bluePose3d.getY());
            Translation3d blueRobotTranslation = bluePose3d.getTranslation();
            Rotation3d blueRobotRotation = bluePose3d.getRotation();

            // TODO: How to clip Pose3d
            Pose3d redRobotTransform = LimelightSim.flipFieldPose(bluePose3d); // FlippingUtil.flipFieldPose(bluePose3d);
            Translation3d redRobotTranslation = redRobotTransform.getTranslation();
            Rotation3d redRobotRotation = redRobotTransform.getRotation();

            double[] bluePose = new double[]{ 
                blueRobotTranslation.getX(), blueRobotTranslation.getY(), blueRobotTranslation.getZ(), 
                Units.radiansToDegrees(blueRobotRotation.getX()), Units.radiansToDegrees(blueRobotRotation.getY()), Units.radiansToDegrees(blueRobotRotation.getZ()),
                latency,
                tagCount,
                0.0, // Tag span?
                tagAverageArea,
                tagAverageDistance  
            };
            double[] redPose = new double[]{ 
                redRobotTranslation.getX(), redRobotTranslation.getY(), redRobotTranslation.getZ(), 
                Units.radiansToDegrees(redRobotRotation.getX()), Units.radiansToDegrees(redRobotRotation.getY()), Units.radiansToDegrees(redRobotRotation.getZ()),
                latency,
                tagCount,
                0.0, // Tag span?
                tagAverageArea,
                tagAverageDistance  
            };
            botpose.set(bluePose);
            botpose_wpiblue.set(bluePose);
            botpose_wpired.set(redPose);

            botpose_orb.set(bluePose);
            botpose_orb_wpiblue.set(bluePose);
            botpose_orb_wpired.set(redPose);

            // TODO: calculate these
            double[] emptyPose = new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latency, tagCount, 0.0, tagAverageArea, tagAverageDistance};
            camerapose_targetspace.set(emptyPose);
            targetpose_cameraspace.set(emptyPose);
            targetpose_robotspace.set(emptyPose);
            botpose_targetspace.set(emptyPose);
            camerapose_robotspace.set(emptyPose);
        }
    }
}

