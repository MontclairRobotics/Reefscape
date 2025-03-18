package frc.robot.vision;

import java.io.IOException;
import java.util.ArrayList;
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
import org.photonvision.targeting.TargetCorner;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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

import frc.robot.vision.LimelightHelpers.LimelightResults;

public class LimelightSim extends SubsystemBase {

    public static enum LimelightModel {
        LIMELIGHT_1(62.5, 56.2),
        LIMELIGHT_2(62.5, 56.2),
        LIMELIGHT_3(62.5, 56.2),
        LIMELIGHT_3G(82, 56.2),
        LIMELIGHT_4(82, 56.2);

        public double hFov;
        public double vFov;

        private LimelightModel(double hFov, double vFov) {
            this.hFov = hFov;
            this.vFov = vFov;
        }
    }
    public static enum LimelightResolution {

        RESOLUTION_320x200(320, 200),
        RESOLUTION_640x480(640, 480),
        RESOLTUION_1280x960(1280, 960);

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
    public static AprilTagFieldLayout loadDefaultFieldLayout() {
        try {
            return AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
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
        private DoubleArrayPublisher rawfiducials;
        private DoubleArrayPublisher tcornxy;

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
            cameraProp.setCalibration(resolution.x, resolution.y, Rotation2d.fromDegrees(model.hFov));
            // Approximate detection noise with average and standard deviation error in pixels.
            // cameraProp.setCalibError(0.25, 0.08);
            cameraProp.setCalibError(0.0, 0.0);
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
            rawfiducials = limelightTable.getDoubleArrayTopic("rawfiducials").publish();
            tcornxy = limelightTable.getDoubleArrayTopic("tcornxy").publish();
            

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

            long tagId = priorityid.get(-1);
            
            if (!results.isEmpty()) {
                PhotonPipelineResult result = results.get(results.size()-1);
                frameCounter++;

                // Timestamp
                double timestamp = result.getTimestampSeconds();

                // Do we have a target?
                if (result.hasTargets()) {
                    tv.set(1);

                    LimelightResults limelightResults =  new LimelightResults();

                    PhotonTrackedTarget bestTarget = result.getBestTarget();
                    if (tagId > 0) {
                        bestTarget = result.getTargets().stream().filter(t -> t.getFiducialId() == tagId).findFirst().orElse(bestTarget);
                    }

                    // TODO: is this correct timestamp to use?
                    double latency = Timer.getFPGATimestamp() - timestamp;
                    int tagCount = result.getTargets().size();
                    double tagAverageArea = result.getTargets().stream().mapToDouble(t -> t.getArea()).average().orElse(0.0);
                    double tagAverageDistance = result.getTargets().stream().mapToDouble(t -> t.bestCameraToTarget.getTranslation().getDistance(Translation3d.kZero)).average().orElse(0.0);

                    // System.out.println("Target: " + result.hasTargets());
                    tx.set(bestTarget.getYaw());
                    txnc.set(bestTarget.getYaw());
                    ty.set(bestTarget.getPitch());
                    tync.set(bestTarget.getPitch());
                    ta.set(bestTarget.getArea());
                    tl.set(latency/2); // Divide latency equally between camera and limelight
                    cl.set(latency/2);
                    t2d.set(new double[]{ 
                        1.0, 
                        tagCount, 
                        latency,
                        0.0,
                        bestTarget.getYaw(),
                        bestTarget.getPitch(),
                        bestTarget.getYaw(),
                        bestTarget.getPitch(),
                        bestTarget.getArea(),
                        bestTarget.getFiducialId(),
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
                    });
                    getpipe.set(0); // Dummy value
                    getpipetype.set("apriltag"); // Dummy value
                    hb.set(frameCounter);
                    hw.set(new double[]{ cameraProperties.getFPS(), 0.0, 0.0, 0.0, 0.0 }); // No values for temp and cpu

                    // Assuming the best is the main one of multi-tag
                    tid.set(bestTarget.getFiducialId());

                    double[] rawfiducialsArray = new double[result.getTargets().size() * 7];
                    double[] tcornxyArray = new double[result.getTargets().size() * 8];
                    for (var i=0; i<result.getTargets().size(); i++) {
                        PhotonTrackedTarget target = result.getTargets().get(i);
                        rawfiducialsArray[7*i] = target.getFiducialId();
                        rawfiducialsArray[7*i+1] = target.getYaw();
                        rawfiducialsArray[7*i+2] = target.getPitch();
                        rawfiducialsArray[7*i+3] = target.getArea();
                        rawfiducialsArray[7*i+4] = target.getBestCameraToTarget().getTranslation().getDistance(Translation3d.kZero);
                        rawfiducialsArray[7*i+5] = target.getBestCameraToTarget().getTranslation().plus(robotToCamera.getTranslation()).getDistance(Translation3d.kZero);
                        rawfiducialsArray[7*i+6] = target.getPoseAmbiguity();
                        // if (target.getFiducialId() == 18 && name.equals("Limelight-Right")) {
                        //     System.out.println("Target 18: " + target.getYaw() + " " + target.getPitch() + " " + target.getArea());
                        // }

                        // if (target.getFiducialId() == 18 && name.equals("Limelight-Right")) {
                        //     System.out.println(target.detectedCorners.size());
                        // }
                        for (int j=0; j<target.detectedCorners.size(); j++) {
                            TargetCorner corner = target.detectedCorners.get(j);
                            // if (target.getFiducialId() == 18 && name.equals("Limelight-Right")) {
                            //     System.out.println(corner.x + ", " + corner.y);
                            // }
                            tcornxyArray[8*i+2*j] = corner.x;
                            tcornxyArray[8*i+2*j+1] = corner.y;
                        }
                    }
                    tcornxy.set(tcornxyArray);
                    rawfiducials.set(rawfiducialsArray);

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

                            double diff = odometryRobotPose.getTranslation().getDistance(blueRobotPose.getTranslation().toTranslation2d());
                            if (diff < 0.1) {
                                publishRobotPose(blueRobotPose, latency, tagCount, tagAverageArea, tagAverageDistance, rawfiducialsArray);

                                stddevs.set(new double[]{0.1, 0.1, 1000.0});
                            }
                        } else {
                            Pose3d bestBlueRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(bestTarget.getBestCameraToTarget(), limelightSim.fieldLayout.getTagPose(bestTarget.getFiducialId()).get(), robotToCamera.inverse());
                            Pose3d altBlueRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(bestTarget.getAlternateCameraToTarget(), limelightSim.fieldLayout.getTagPose(bestTarget.getFiducialId()).get(), robotToCamera.inverse());
                            
                            double bestDiff = odometryRobotPose.getTranslation().getDistance(bestBlueRobotPose.getTranslation().toTranslation2d());
                            double altDiff = odometryRobotPose.getTranslation().getDistance(altBlueRobotPose.getTranslation().toTranslation2d());

                            if (bestDiff < altDiff) {
                                if (bestDiff < 0.1) {
                                    publishRobotPose(bestBlueRobotPose, latency, tagCount, tagAverageArea, tagAverageDistance, rawfiducialsArray);
                                    
                                    // Higher stddevs for the non multi-tag case
                                    stddevs.set(new double[]{0.5, 0.5, 1000.0});
                                }
                            } else {
                                if (altDiff < 0.1) {
                                    publishRobotPose(altBlueRobotPose, latency, tagCount, tagAverageArea, tagAverageDistance, rawfiducialsArray);

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
        public void publishRobotPose(Pose3d bluePose3d, double latency, int tagCount, double tagAverageArea, double tagAverageDistance, double[] rawfiducials) {
            // System.out.println("publishRobotPose: " + bluePose3d.getX() + " " + bluePose3d.getY());
            Translation3d blueRobotTranslation = bluePose3d.getTranslation();
            Rotation3d blueRobotRotation = bluePose3d.getRotation();

            // TODO: How to clip Pose3d
            // TODO: ONLY WORKS ON BLUE
            Pose3d redRobotTransform = bluePose3d; // PoseUtils.flipFieldPose(bluePose3d); // FlippingUtil.flipFieldPose(bluePose3d);
            Translation3d redRobotTranslation = redRobotTransform.getTranslation();
            Rotation3d redRobotRotation = redRobotTransform.getRotation();

            double[] bluePose = new double[11 + rawfiducials.length];
            bluePose[0] = blueRobotTranslation.getX();
            bluePose[1] = blueRobotTranslation.getY();
            bluePose[2] = blueRobotTranslation.getZ();
            bluePose[3] = Units.radiansToDegrees(blueRobotRotation.getX());
            bluePose[4] = Units.radiansToDegrees(blueRobotRotation.getY());
            bluePose[5] = Units.radiansToDegrees(blueRobotRotation.getZ());
            bluePose[6] = latency;
            bluePose[7] = tagCount;
            bluePose[8] = 0.0; // Tag span?
            bluePose[9] = tagAverageArea;
            bluePose[10] = tagAverageDistance;
            for (int i=0; i<rawfiducials.length; i++) {
                bluePose[11+i] = rawfiducials[i];
            }

            double[] redPose = new double[11 + rawfiducials.length];
            redPose[0] = redRobotTranslation.getX();
            redPose[1] = redRobotTranslation.getY();
            redPose[2] = redRobotTranslation.getZ();
            redPose[3] = Units.radiansToDegrees(redRobotRotation.getX());
            redPose[4] = Units.radiansToDegrees(redRobotRotation.getY());
            redPose[5] = Units.radiansToDegrees(redRobotRotation.getZ());
            redPose[6] = latency;
            redPose[7] = tagCount;
            redPose[8] = 0.0; // Tag span?
            redPose[9] = tagAverageArea;
            redPose[10] = tagAverageDistance;
            for (int i=0; i<rawfiducials.length; i++) {
                redPose[11+i] = rawfiducials[i];
            }
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

