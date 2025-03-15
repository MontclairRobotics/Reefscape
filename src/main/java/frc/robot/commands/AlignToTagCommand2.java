package frc.robot.commands;

import java.io.IOException;
import java.util.HashMap;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.vision.LimelightSim;

/**
 * This command will align to the left, right or center of a tag
 */
public class AlignToTagCommand2 extends Command {
    
    // Focal lengths in pixels
    // Assumes resolution of 1280x720 and Limelight 4 with FOV of 82 horizonatl and 52.8 vertical
    public static final double HORIZONTAL_FOCAL_LENGTH = (1280.0 / 2.0) / Math.tan(Units.degreesToRadians(82.0 / 2.0));
    public static final double VERTICAL_FOCAL_LENGTH = (720.0 / 2.0) / Math.tan(Units.degreesToRadians(52.8 / 2.0));

    public static double REEF_APRIL_TAG_HEIGHT = Units.inchesToMeters(12);

    private PIDController xController;
    private PIDController yController;
    private PIDController thetaController;

    private Limelight camera;
    private int assignedTagId = -1;
    private int tagId = -1;
    private boolean useTagSize = false;
    private TagOffset tagOffset;
    private Rotation2d targetRotation;

    
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
    private AlignToTagCommand2(Limelight camera, boolean useTagSize) {
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
    }
    /**
     * Aligns to the nearest or largest tag of the given type that's visible
     * @param camera The limelight to use
     * @param type The type of tag to align to
     * @param offset The offset to align to
     * @param useTagSize If true, uses the size of the tag to determine distance, otherwise uses the height of the tag
     */
    public AlignToTagCommand2(Limelight camera, TagOffset offset, boolean useTagSize) {
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
    public AlignToTagCommand2(Limelight camera, int tagId, TagOffset offset, boolean useTagSize) {
        this(camera, useTagSize);
        this.assignedTagId = tagId;
        this.tagOffset = offset;
    }

    /**
     * Finds tag to go to and sets the setpoints for the PID controllers
     */
    @Override
    public void initialize() {
        System.out.println("initializing align to tag command");
        // Find largest tag
        if (assignedTagId != -1) {
            tagId = assignedTagId;
        } else {
            tagId = camera.getLargestAprilTag(tagRotationsMap.keySet());
        }
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

        Logger.recordOutput("Align2/xSetpoint", tagOffset.getXOffsetM() - camera.getCameraOffsetX());
        Logger.recordOutput("Align2/ySetpoint", camera.getCameraOffsetY() + tagOffset.getYOffsetM());
        Logger.recordOutput("Align2/thetaSetpoint", targetRotation.getDegrees());
    }

    @Override
    public void execute() {
        if (tagId == -1) {
            return;
        }
        if (camera.getTagID() != tagId) {
            return;
        }

        // Get current rotation and PID for it
        Optional<Pose2d> optRobotPose = RobotContainer.drivetrain.getPoseAtTime(camera.getTimestampSeconds());
        if (optRobotPose.isEmpty()) {
            return;
        }
        Pose2d robotPose = optRobotPose.get();

        double currentTheta = RobotContainer.drivetrain.getRobotPose().getRotation().getRadians();
        double thenTheta = robotPose.getRotation().getRadians();
        double thetaSpeed = thetaController.calculate(currentTheta);

        // The trigonometry here is dependent on the robot being parallel to the tag
        // So first, do the rotation until we are parallel
        // If for some reason we lose the tag, we will just rotate until we find it again (hopefully?)
        // Should command end if we don't see the tag?
        // Publish rotations
        Logger.recordOutput("Align2/currentTheta", Units.radiansToDegrees(currentTheta));
        Logger.recordOutput("Align2/thenTheta", Units.radiansToDegrees(thenTheta));
        Logger.recordOutput("Align2/thetaSpeed", Units.radiansToDegrees(thetaSpeed));

    
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

        Logger.recordOutput("Align2/minX1", minX);
        Logger.recordOutput("Align2/maxX1", maxX);
        double minXOffset = - (maxX - 1280 / 2);
        double maxXOffset = - (minX - 1280 / 2);
        Logger.recordOutput("Align2/minX", minXOffset);
        Logger.recordOutput("Align2/maxX", maxXOffset);
        double minXAngle = Math.atan(minXOffset / HORIZONTAL_FOCAL_LENGTH);
        double maxXAngle = Math.atan(maxXOffset / HORIZONTAL_FOCAL_LENGTH);
        Logger.recordOutput("Align2/minXAngle", Units.radiansToDegrees(minXAngle));
        Logger.recordOutput("Align2/maxXAngle", Units.radiansToDegrees(maxXAngle));

        double robotToTagAngle = Units.degreesToRadians(90) - (thenTheta - targetRotation.getRadians());
        double minXToTagAngle = robotToTagAngle - minXAngle;
        double maxXToTagAngle = robotToTagAngle - maxXAngle;
        double maxXToMinXAngle = maxXAngle - minXAngle;
        Logger.recordOutput("Align2/robotToTagAngle", Units.radiansToDegrees(robotToTagAngle));
        Logger.recordOutput("Align2/minXToTagAngle", Units.radiansToDegrees(minXToTagAngle));
        Logger.recordOutput("Align2/maxXToTagAngle", Units.radiansToDegrees(maxXToTagAngle));
        Logger.recordOutput("Align2/maxXToMinXAngle", Units.radiansToDegrees(maxXToMinXAngle));

        double distanceToMinX = Units.inchesToMeters(8.75) * Math.sin(maxXToTagAngle) / Math.sin(maxXToMinXAngle);
        double distanceToMaxX = Units.inchesToMeters(8.75) * Math.sin(Units.degreesToRadians(180) - minXToTagAngle) / Math.sin(maxXToMinXAngle);
        Logger.recordOutput("Align2/distanceToMinX", distanceToMinX);
        Logger.recordOutput("Align2/distanceToMaxX", distanceToMaxX);

        double robotToMinXAngle = robotToTagAngle - minXToTagAngle;
        double robotToXAngle = robotToMinXAngle + maxXToMinXAngle / 2;
        Logger.recordOutput("Align2/robotToMinXAngle", Units.radiansToDegrees(robotToMinXAngle));
        Logger.recordOutput("Align2/robotToXAngle", Units.radiansToDegrees(robotToXAngle));

        
        double y = distanceToMinX * Math.sin(minXToTagAngle) / Math.sin(Units.degreesToRadians(180) - robotToTagAngle);
        double x = Math.sin(robotToXAngle) * distanceToMinX;

        // PID for x and y speeds
        double xSpeed = xController.calculate(x);
        double ySpeed = yController.calculate(y);

        // Publish to network tables
        Logger.recordOutput("Align2/x", x);
        Logger.recordOutput("Align2/y", y);
        Logger.recordOutput("Align2/xSpeed", xSpeed);
        Logger.recordOutput("Align2/ySpeed", ySpeed);

        // x and y are robot relative, but we want to move field relative
        Pose3d tagPose = fieldLayout.getTagPose(tagId).get();

        // double poseX = tagPose.getX() - x * Math.sin(Math.PI/2 - currentTheta) - y * Math.cos(currentTheta);
        double poseX = tagPose.getX() - (x + camera.getCameraOffsetX()) * Math.cos(currentTheta) + (y - camera.getCameraOffsetY()) * Math.sin(currentTheta);
        double poseY = tagPose.getY() + (y - camera.getCameraOffsetY()) * Math.cos(currentTheta) - (x + camera.getCameraOffsetX()) * Math.sin(currentTheta);
        Pose3d alignPose = new Pose3d(new Pose2d(poseX, poseY, Rotation2d.fromRadians(currentTheta)));
        alignPose = alignPose.transformBy(new Transform3d(0, 0, 1, Rotation3d.kZero));



        Translation2d tagPosition = LimelightSim.loadDefaultFieldLayout().getTagPose(18).get().toPose2d().getTranslation();
        Translation2d robotPosition = robotPose.getTranslation();
        double x2 = Math.abs(robotPosition.getX() - tagPosition.getX());
        double y2 = Math.abs(robotPosition.getY() - tagPosition.getY());
        Logger.recordOutput("Align2/x2", x2);
        Logger.recordOutput("Align2/y2", y2);
        Logger.recordOutput("Align2/dist2", Math.sqrt(x2*x2 + y2*y2));

        // We do this robot relative instead of field relative
        // x and y are robot relative
        // X is forward and back robot relative 
        // Y is left and right robot relative
        // We are already facing the tag, so we don't need to rotate
        // RobotContainer.drivetrain.drive(-xSpeed, ySpeed, 0, false, false);
    }

    @Override
    public void end(boolean interrupted) {
        tagId = -1;
        RobotContainer.drivetrain.drive(0, 0, 0, true, false);
    }

    @Override
    public boolean isFinished() {
        // Should command end if we don't see the tag?
        boolean isFinished = xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
        return false;
    }
}
