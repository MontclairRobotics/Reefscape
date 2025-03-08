package frc.robot.commands;

import java.io.IOException;
import java.util.HashMap;
import java.util.Set;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.TagOffset;
import frc.robot.vision.Limelight;

/**
 * This command will align to the left, right or center of a tag
 */
public class AlignToTagCommand extends Command {
    
    // Focal lengths in pixels
    // Assumes resolution of 1280x720 and Limelight 4 with FOV of 82 horizonatl and 52.8 vertical
    public static final double HORIZONTAL_FOCAL_LENGTH = (1280.0 / 2.0) / Math.tan(Units.degreesToRadians(82.0 / 2.0));
    public static final double VERTICAL_FOCAL_LENGTH = (720.0 / 2.0) / Math.tan(Units.degreesToRadians(52.8 / 2.0));

    public static double REEF_APRIL_TAG_HEIGHT = Units.inchesToMeters(12);

    private PIDController xController;
    private PIDController yController;
    private PIDController thetaController;

    private Limelight camera;
    private int tagId = -1;
    private boolean useTagSize = false;
    private TagOffset tagOffset;
    private Rotation2d targetRotation;

    DoublePublisher xPub;
    DoublePublisher yPub;
    DoublePublisher rotPub;

    DoublePublisher xOutputPub;
    DoublePublisher yOutputPub;
    DoublePublisher rotOutputPub;

    DoublePublisher xSetpointPub;
    DoublePublisher ySetpointPub;
    DoublePublisher rotSetpointPub;

    StructPublisher<Pose3d> tagPub;
    StructPublisher<Pose3d> posePub;

    BooleanPublisher isFinishedPub;

    IntegerPublisher idPub;

    
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

    // Field
    public static AprilTagFieldLayout fieldLayout = loadDefaultFieldLayout();
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
     * Aligns to the given tag
     * @param camera The limelight to use
     * @param useTagSize If true, uses the size of the tag to determine distance, otherwise uses the height of the tag
     */
    private AlignToTagCommand(Limelight camera, boolean useTagSize) {
        this.camera = camera;
        this.useTagSize = useTagSize;

        xController = new PIDController(5, 0, .1);
        xController.setTolerance(0.02); // 2cm
        yController = new PIDController(10, 0, 0);
        yController.setTolerance(0.02); // 2cm
        thetaController = RobotContainer.drivetrain.thetaController;

        addRequirements(RobotContainer.drivetrain);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable alignCommandTable = inst.getTable("AlignToAprilTagCommandOffset");
        xPub = alignCommandTable.getDoubleTopic("X").publish();
        yPub = alignCommandTable.getDoubleTopic("Y").publish();
        rotPub = alignCommandTable.getDoubleTopic("Rotation").publish();
        xOutputPub = alignCommandTable.getDoubleTopic("X Output").publish();
        yOutputPub = alignCommandTable.getDoubleTopic("Y Output").publish();
        rotOutputPub = alignCommandTable.getDoubleTopic("Rotation Output").publish();
        xSetpointPub = alignCommandTable.getDoubleTopic("X Setpoint").publish();
        ySetpointPub = alignCommandTable.getDoubleTopic("Y Setpoint").publish();
        rotSetpointPub = alignCommandTable.getDoubleTopic("Rotation Setpoint").publish();
        idPub = alignCommandTable.getIntegerTopic("ID").publish();
        posePub = alignCommandTable.getStructTopic("Pose", Pose3d.struct).publish();
        tagPub = alignCommandTable.getStructTopic("TagPose", Pose3d.struct).publish();
        isFinishedPub = alignCommandTable.getBooleanTopic("Is Finished").publish();
    }
    /**
     * Aligns to the nearest or largest tag of the given type that's visible
     * @param camera The limelight to use
     * @param type The type of tag to align to
     * @param offset The offset to align to
     * @param useTagSize If true, uses the size of the tag to determine distance, otherwise uses the height of the tag
     */
    public AlignToTagCommand(Limelight camera, TagOffset offset, boolean useTagSize) {
        this(camera, useTagSize);
        this.tagOffset = offset;
    }

    /**
     * Aligns to the given tag
     * @param camera The limelight to use
     * @param tagId The ID of the tag to align to
     * @param offset The offset to align to
     * @param useTagSize If true, uses the size of the tag to determine distance, otherwise uses the height of the tag
     */
    public AlignToTagCommand(Limelight camera, int tagId, TagOffset offset, boolean useTagSize) {
        this(camera, useTagSize);
        this.tagOffset = offset;
    }

    /**
     * Finds tag to go to and sets the setpoints for the PID controllers
     */
    @Override
    public void initialize() {
        System.out.println("initializing align to tag command");
        // Find largest tag
        tagId = camera.getLargestAprilTag(tagRotationsMap.keySet());
        if (tagId == -1) {
            System.out.println("No tag found");
            return;
        }
        this.targetRotation = tagRotationsMap.get(tagId);

        // Set limelight priority tag
        camera.setPriorityTagID(tagId);

        xController.setSetpoint(tagOffset.getXOffsetM() - camera.getCameraOffsetX());
        yController.setSetpoint(camera.getCameraOffsetY() + tagOffset.getYOffsetM());
        thetaController.setSetpoint(targetRotation.getRadians());

        xSetpointPub.set(tagOffset.getXOffsetM() - camera.getCameraOffsetX());
        ySetpointPub.set(camera.getCameraOffsetY() + tagOffset.getYOffsetM());
        rotSetpointPub.set(targetRotation.getDegrees());
    }

    @Override
    public void execute() {
        if (tagId == -1) {
            return;
        }

        // Get current rotation and PID for it
        double currentTheta = RobotContainer.drivetrain.getRobotPose().getRotation().getRadians();
        double thetaSpeed = thetaController.calculate(currentTheta);

        // The trigonometry here is dependent on the robot being parallel to the tag
        // So first, do the rotation until we are parallel
        // If for some reason we lose the tag, we will just rotate until we find it again (hopefully?)
        // Should command end if we don't see the tag?
        if (!thetaController.atSetpoint() || camera.getTagID() != tagId) {
            // Publish rotations
            rotPub.set(Units.radiansToDegrees(currentTheta));
            rotOutputPub.set(Units.radiansToDegrees(thetaSpeed));

            // Turn without movement field relative
            RobotContainer.drivetrain.drive(0, 0, thetaSpeed, true, false);
        } else {

            // Calculate x and y distances to tag
            // x and y are robot relative
            // To robot x is forward and back, y is left and right
            double x, y;
            
            // Both of these methods only work when we are parallel to the tag
            if (!useTagSize) {
                // This method gets the y angle of the tag
                // And uses that with height of tag and camera in real world to determine distance

                // x distance to tag = height of tag / tan(angle to tag)
                // height of tag is difference betwene tag and camera
                // angle to tag is angle from camera minus angle the camera is at
                x = REEF_APRIL_TAG_HEIGHT - camera.getCameraHeightMeters() / Math.tan(Units.degreesToRadians(camera.getTY() - camera.getCameraAngle()));
                // y distance to tag is x distance * tan(x angle to tag)
                y = x * Math.tan(Units.degreesToRadians(camera.getTX()));
            } else {
                // This method gets the corners of the tag
                // And uses the height of the tag in pixels to determine distance

                // Get corners of tag
                double[] corners = camera.getCorners(tagId);

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
                // Width and height of tag as angles (radians)
                double xAngle = Math.atan(width / HORIZONTAL_FOCAL_LENGTH);
                double yAngle = Math.atan(height / VERTICAL_FOCAL_LENGTH);
                
                // Size of tags are 6.5 inches
                // So x distance to tag = size / tan()
                x = Math.abs(Units.inchesToMeters(6.5) / Math.tan(yAngle));
                // y distance to tag is x distance * tan(x angle to tag)
                y = x * Math.tan(Units.degreesToRadians(camera.getTX()));
            }
    
            // PID for x and y speeds
            double xSpeed = xController.calculate(x);
            double ySpeed = yController.calculate(y);

            // Publish to network tables
            xPub.set(x);
            yPub.set(y);
            rotPub.set(Units.radiansToDegrees(currentTheta));
            xOutputPub.set(xSpeed);
            yOutputPub.set(ySpeed);
            rotOutputPub.set(Units.radiansToDegrees(thetaSpeed));

            // x and y are robot relative, but we want to move field relative
            Pose3d tagPose = fieldLayout.getTagPose(tagId).get();
            tagPub.set(tagPose);

            // double poseX = tagPose.getX() - x * Math.sin(Math.PI/2 - currentTheta) - y * Math.cos(currentTheta);
            double poseX = tagPose.getX() - (x + camera.getCameraOffsetX()) * Math.cos(currentTheta) + (y - camera.getCameraOffsetY()) * Math.sin(currentTheta);
            double poseY = tagPose.getY() + (y - camera.getCameraOffsetY()) * Math.cos(currentTheta) - (x + camera.getCameraOffsetX()) * Math.sin(currentTheta);
            Pose3d alignPose = new Pose3d(new Pose2d(poseX, poseY, Rotation2d.fromRadians(currentTheta)));
            alignPose = alignPose.transformBy(new Transform3d(0, 0, 1, Rotation3d.kZero));

            posePub.set(alignPose);

            // We do this robot relative instead of field relative
            // x and y are robot relative
            // X is forward and back robot relative 
            // Y is left and right robot relative
            // We are already facing the tag, so we don't need to rotate
            RobotContainer.drivetrain.drive(-xSpeed, ySpeed, 0, false, false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        tagId = -1;
        RobotContainer.drivetrain.drive(0, 0, 0, true, false);
    }

    @Override
    public boolean isFinished() {
        // Should command end if we don't see the tag?
        boolean isFinished = xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint() && camera.hasValidTarget();
        isFinishedPub.set(isFinished);
        return false;
    }
}
