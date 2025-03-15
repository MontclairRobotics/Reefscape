package frc.robot.commands;

import java.io.IOException;
import java.util.HashMap;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.TagOffset;
import frc.robot.vision.Limelight;

/**
 * This command will align to the left, right or center of a tag
 */
public class AlignToTagCommand4 extends Command {
    
    // Focal lengths in pixels
    // Assumes resolution of 1280x720 and Limelight 4 with FOV of 82 horizonatl and 52.8 vertical
    public static final double HORIZONTAL_FOCAL_LENGTH = (1280.0 / 2.0) / Math.tan(Units.degreesToRadians(82.0 / 2.0));
    public static final double VERTICAL_FOCAL_LENGTH = (800.0 / 2.0) / Math.tan(Units.degreesToRadians(52.8 / 2.0));

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
    private boolean isOffset;
    
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
    private AlignToTagCommand4(TagOffset offset, boolean useTagSize, boolean isOffset) {
        if (offset == TagOffset.LEFT) {
            this.camera = RobotContainer.rightLimelight;
        } else {
            this.camera = RobotContainer.leftLimelight;
        }
        this.tagOffset = offset;
        this.useTagSize = useTagSize;
        this.isOffset = isOffset;

        xController = new PIDController(4.5, 0, .1);
        xController.setTolerance(0.02); // 2cm
        yController = new PIDController(3.0, 0, .05);
        yController.setTolerance(0.02); // 2cm
        thetaController = RobotContainer.drivetrain.thetaController;

        addRequirements(RobotContainer.drivetrain);
    }

    /**
     * Aligns to the given tag
     * @param camera The limelight to use
     * @param tagId The ID of the tag to align to
     * @param offset The offset to align to
     * @param useTagSize If true, uses the size of the tag to determine distance, otherwise uses the height of the tag
     */
    public AlignToTagCommand4(int tagId, TagOffset offset, boolean useTagSize, boolean isOffset) {
        this(offset, useTagSize, isOffset);
        this.assignedTagId = tagId;
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

        // x is forward and back, y is side to side
        double xSetpoint = tagOffset.getXOffsetM() - camera.getCameraOffsetX() + (isOffset ? 0.3 : 0.0);
        double ySetpoint = camera.getCameraOffsetY() + tagOffset.getYOffsetM();
        double thetaSetpoint = targetRotation.getRadians();
        xController.setSetpoint(xSetpoint);
        yController.setSetpoint(ySetpoint);
        thetaController.setSetpoint(thetaSetpoint);

        Logger.recordOutput("AlignToTag/xSetpoint", xSetpoint);
        Logger.recordOutput("AlignToTag/ySetpoint", ySetpoint);
        Logger.recordOutput("AlignToTag/thetaSetpoint", Units.radiansToDegrees(thetaSetpoint));
    }

    /**
     * This method gets the y angle of the tag
     * And uses that with height of tag and camera in real world to determine distanced
     */
    private Translation2d getXYFromTxTy(double tx, double ty) {

        // x distance to tag = height of tag / tan(angle to tag)
        // height of tag is difference betwene tag and camera
        // angle to tag is angle from camera minus angle the camera is at
        double x = Math.abs(REEF_APRIL_TAG_HEIGHT - camera.getCameraHeightMeters()) / Math.tan(Units.degreesToRadians(ty - camera.getCameraAngle()));
        // y distance to tag is x distance * tan(x angle to tag)
        double y = x * Math.tan(Units.degreesToRadians(tx));
        return new Translation2d(x, y);
    }

    /**
     * This method gets the corners of the tag
     * And uses the height of the tag in pixels to determine distance
     */
    private Translation2d getXYFromCorners(double tx, double ty) {
        // This method gets the corners of the tag
        // And uses the height of the tag in pixels to determine distance

        // Get corners of tag
        double[] corners = camera.getCorners(tagId);
        if (corners == null) {
            return getXYFromTxTy(tx, ty);
        }

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
        Logger.recordOutput("AlignToTag/width", width);
        Logger.recordOutput("AlignToTag/height", height);

        // Width and height of tag as angles (radians)
        double xAngle = Math.atan(width / HORIZONTAL_FOCAL_LENGTH);
        double yAngle = Math.atan(height / VERTICAL_FOCAL_LENGTH);
        Logger.recordOutput("AlignToTag/xAngle", xAngle);
        Logger.recordOutput("AlignToTag/yAngle", yAngle);
        
        // Size of tags are 8.75 inches
        // So x distance to tag = size / tan()
        double x = Math.abs(Units.inchesToMeters(8.75) / Math.tan(yAngle));
        // y distance to tag is x distance * tan(x angle to tag)
        double y = x * Math.tan(Units.degreesToRadians(tx));

        return new Translation2d(x, y);
    }

    @Override
    public void execute() {
        if (tagId == -1) {
            return;
        }
        if (camera.getTagID() != tagId) {
            return;
        }

        // Get robot pose
        Pose2d robotPose = RobotContainer.drivetrain.getRobotPose();

        // Get tx and ty
        double tx = camera.getTX();
        double ty = camera.getTY();
        Logger.recordOutput("AlignToTag/tx", tx);
        Logger.recordOutput("AlignToTag/ty", ty);

        // Calculate x and y distances to tag
        // x and y are robot relative
        // To robot x is forward and back, y is left and right
        Translation2d position = useTagSize ? getXYFromCorners(tx, ty) : getXYFromTxTy(tx, ty);
        double x = position.getX();
        double y = position.getY();
        Logger.recordOutput("AlignToTag/x", x);
        Logger.recordOutput("AlignToTag/y", y);

        // Total distance (this isn't 100% correct because x and y are not perpendicular all the time)
        // But close enough for what we need
        double distance = Math.hypot(x, y);
        Logger.recordOutput("AlignToTag/distance", Units.radiansToDegrees(distance));

        double robotToTagAngle = robotPose.getRotation().getRadians() - targetRotation.getRadians();
        Logger.recordOutput("AlignToTag/robotToTagAngle", Units.radiansToDegrees(robotToTagAngle));

        // Get sideways (relative to tag) distance
        double sidewaysDistance = Math.abs(Math.sin(robotToTagAngle) * x - Math.cos(robotToTagAngle) * y);
        Logger.recordOutput("AlignToTag/sidewaysDistance", sidewaysDistance);

        // Get ratio to use for x and y setpoints
        // This will curve the path to make robot face tag as it comes towards it
        double ratio = isOffset ? Math.min(sidewaysDistance, 1) : 0.0;
        Logger.recordOutput("AlignToTag/ratio", ratio);

        // Set x and y setpoints
        // PID for x and y speeds
        double xSetpoint = tagOffset.getXOffsetM() - camera.getCameraOffsetX() + (isOffset ? 0.3 : 0.0);
        double ySetpoint = camera.getCameraOffsetY() + tagOffset.getYOffsetM();
        double dynamicXSetpoint = xSetpoint + Math.cos(robotToTagAngle) * ratio * distance * 0.5;
        double dynamicYSetpoint = ySetpoint + Math.sin(robotToTagAngle) * ratio * distance * 0.5;
        xController.setSetpoint(dynamicXSetpoint);
        yController.setSetpoint(dynamicYSetpoint);
        double xSpeed = xController.calculate(x);
        double ySpeed = yController.calculate(y);
        Logger.recordOutput("AlignToTag/xSpeed", xSpeed);
        Logger.recordOutput("AlignToTag/ySpeed", ySpeed);
        Logger.recordOutput("AlignToTag/dynamicXSetpoint", dynamicXSetpoint);
        Logger.recordOutput("AlignToTag/dynamicYSetpoint", dynamicYSetpoint);


        // Get current rotation and PID for it
        double currentTheta = robotPose.getRotation().getRadians();
        // But if tag is off to the side and we are turning away from it, we need to correct
        // So we don't lose sight of tag
        double txError = 0;
        if ((tx - 4.135) > 0) {
            txError = Math.max((tx - 4.135) - 10, 0);
        } else {
            txError = Math.min((tx - 4.135) + 10, 0);
        }
        if (Math.signum(currentTheta - targetRotation.getRadians()) == Math.signum(txError)) {
            txError = 0;
        }
        double dynamicThetaSetpoint = targetRotation.getRadians() - Units.degreesToRadians(txError);
        thetaController.setSetpoint(dynamicThetaSetpoint);
        double thetaSpeed = thetaController.calculate(currentTheta);
        Logger.recordOutput("AlignToTag/currentTheta", Units.radiansToDegrees(currentTheta));
        Logger.recordOutput("AlignToTag/dynamicThetaSetpoint", Units.radiansToDegrees(dynamicThetaSetpoint));
        Logger.recordOutput("AlignToTag/thetaSpeed", Units.radiansToDegrees(thetaSpeed));



        // We do this robot relative instead of field relative
        // x and y are robot relative
        // X is forward and back robot relative 
        // Y is left and right robot relative
        // We are already facing the tag, so we don't need to rotate
        RobotContainer.drivetrain.drive(-xSpeed, ySpeed, thetaSpeed, false, false);
    }

    /**
     * Stop the robot on end
     */
    @Override
    public void end(boolean interrupted) {
        System.out.println("AlignToTagCommand4 end");
        tagId = -1;
        RobotContainer.drivetrain.drive(0, 0, 0, true, false);
    }

    /**
     * We are finished if we are at end location and rotation
     * Or if we don't see the tag
     */
    @Override
    public boolean isFinished() {
        double currentTheta = RobotContainer.drivetrain.getRobotPose().getRotation().getRadians();
        boolean atAngle = Math.abs(targetRotation.getRadians() - currentTheta) < Units.degreesToRadians(1);
        boolean atSetPoint = xController.atSetpoint() && yController.atSetpoint() && atAngle;
        boolean cameraSeesTag = camera.hasValidTarget() && camera.getTagID() == tagId;
        boolean isFinished =  atSetPoint || !cameraSeesTag;
        return isFinished;
    }
}
