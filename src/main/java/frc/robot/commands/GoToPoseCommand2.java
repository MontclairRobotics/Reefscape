// package frc.robot.commands;

// import java.io.IOException;

// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.apriltag.AprilTag;
// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.networktables.DoublePublisher;
// import edu.wpi.first.networktables.DoubleTopic;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.units.Units;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.Auto;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.util.PoseUtils;
// import frc.robot.util.TagOffset;
// import frc.robot.util.Tunable;
// import frc.robot.util.TunerConstants;
// import frc.robot.vision.Limelight;

// public class GoToPoseCommand2 extends Command {

//     public static final double OFFSET_FROM_TARGET = 0.5;

//     private PIDController xController;
//     private PIDController yController;
//     private PIDController thetaController;

//     private Pose2d targetPose;
//     private Pose2d offsetTargetPose;
//     private TagOffset direction;

//     DoublePublisher xOutputPub;
//     DoublePublisher yOutputPub;
//     DoublePublisher rotOutputPub;

//     private int tagId;
//     private boolean canSeeTag = false;
//     private Limelight limelight;

//     // DoublePublisher xPosePub;
//     // DoublePublisher yPosePub;
//     // DoublePublisher rotPosePub;

//     // Field
//     public static AprilTagFieldLayout fieldLayout = loadDefaultFieldLayout();
//     /**
//      * Load the field layout from the default field
//      */
//     private static AprilTagFieldLayout loadDefaultFieldLayout() {
//         try {
//             return AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
//         } catch (IOException e) {
//             e.printStackTrace();
//             return null;
//         }
//     }

//     @Override
//     public void initialize() {

//         //defaults to center
//         // if(direction == ScoreDirection.CENTER) {
//             targetPose = RobotContainer.drivetrain.getClosestScoringPose(Drivetrain.BLUE_SCORING_POSES);
//             limelight = RobotContainer.rightLimelight;
//         //}
//         if(direction.isLeft()) {
//             targetPose = RobotContainer.drivetrain.getClosestScoringPose(Drivetrain.LEFT_BLUE_SCORING_POSES);
//             limelight = RobotContainer.rightLimelight;
//         } else if(direction.isRight()) {
//             targetPose = RobotContainer.drivetrain.getClosestScoringPose(Drivetrain.RIGHT_BLUE_SCORING_POSES);
//             limelight = RobotContainer.leftLimelight;
//         } 
        
//         //Cancels the command if for some reason the target pose is null 
//         //(i.e. if the direction is not CENTER, LEFT, or REVERSE)
//         if(targetPose == null) {
//             cancel();
//         }

//         // Get the tag we are aligning up to
//         tagId = getClosestTag(targetPose);

//         // Get offset 0.5 meter back from target
//         Translation2d offset = new Translation2d(-OFFSET_FROM_TARGET, 0).rotateBy(targetPose.getRotation());
//         offsetTargetPose = new Pose2d(targetPose.getTranslation().plus(offset), targetPose.getRotation());

//         // NetworkTableInstance inst = NetworkTableInstance.getDefault();
//         // NetworkTable poseCommandTable = inst.getTable("Pose Command");
//         //.Auto.field.setRobotPose(targetPose);

//         // DoubleTopic xSetpointTopic = poseCommandTable.getDoubleTopic("X Setpoint");
//         // DoublePublisher xSetpointPub = xSetpointTopic.publish();
//         // DoubleTopic ySetpointTopic = poseCommandTable.getDoubleTopic("Y Setpoint");
//         // DoublePublisher ySetpointPub = ySetpointTopic.publish();
//         // DoubleTopic rotSetpointTopic = poseCommandTable.getDoubleTopic("Rot Setpoint");
//         // DoublePublisher rotSetpointPub = rotSetpointTopic.publish();

//         // xSetpointPub.set(targetPose.getX());
//         // ySetpointPub.set(targetPose.getY());
//         // rotSetpointPub.set(targetPose.getRotation().getRadians());

//         // DoubleTopic xOutputTopic = poseCommandTable.getDoubleTopic("X Output");
//         // xOutputPub = xOutputTopic.publish();
//         // DoubleTopic yOutputTopic = poseCommandTable.getDoubleTopic("Y Output");
//         // yOutputPub = yOutputTopic.publish();
//         // DoubleTopic rotOutputTopic = poseCommandTable.getDoubleTopic("Rot Output");
//         // rotOutputPub = rotOutputTopic.publish();

//         // DoubleTopic xPoseTopic = poseCommandTable.getDoubleTopic("X Pose");
//         // xPosePub = xPoseTopic.publish();
//         // DoubleTopic yPoseTopic = poseCommandTable.getDoubleTopic("Y Pose");
//         // yPosePub = yPoseTopic.publish();
//         // DoubleTopic rotPoseTopic = poseCommandTable.getDoubleTopic("Rot Pose");
//         // rotPosePub = rotPoseTopic.publish();
    
//         // If we can't see the tag, set the setpoint to half meter back from goal
//         // until we can see it
//         if (limelight.canSeeTag(tagId)) {
//             canSeeTag = true;
//             xController.setSetpoint(targetPose.getX());
//             yController.setSetpoint(targetPose.getY());
//             thetaController.setSetpoint(targetPose.getRotation().getRadians());
//         } else {
//             canSeeTag = false;
//             xController.setSetpoint(offsetTargetPose.getX());
//             yController.setSetpoint(offsetTargetPose.getY());
//             thetaController.setSetpoint(offsetTargetPose.getRotation().getRadians());
//         }
//     }

//     public GoToPoseCommand2(TagOffset direction) {
//         this.direction = direction; //sets the direction
//         addRequirements(RobotContainer.drivetrain); //requires the drivetrain
//         xController = new PIDController(3.5, 0, .035); //creates the PIDControllers
//         yController = new PIDController(3.5, 0, .035); //TODO tolerances
//         thetaController = RobotContainer.drivetrain.thetaController;
//     }

//     @Override
//     public void execute() {
//         //current pose to PID from
//         Pose2d currentPose = RobotContainer.drivetrain.getState().Pose;

//         // If we can't see the tag, set the setpoint to half meter back from goal
//         // until we can see it
//         // Only switch setpoint if state changed
//         if (limelight.canSeeTag(tagId)) {
//             if (!canSeeTag) {
//                 canSeeTag = true;
//                 xController.setSetpoint(targetPose.getX());
//                 yController.setSetpoint(targetPose.getY());
//                 thetaController.setSetpoint(targetPose.getRotation().getRadians());
//                 }
//         } else {
//             if (canSeeTag) {
//                 canSeeTag = false;
//                 xController.setSetpoint(offsetTargetPose.getX());
//                 yController.setSetpoint(offsetTargetPose.getY());
//                 thetaController.setSetpoint(offsetTargetPose.getRotation().getRadians());
//             }
//         }

//         // //logging
//         // xPosePub.set(currentPose.getX());
//         // yPosePub.set(currentPose.getY());
//         // rotPosePub.set(currentPose.getRotation().getRadians());

//         //calculating outputs
//         double xSpeed = xController.calculate(currentPose.getX());
//         double ySpeed = yController.calculate(currentPose.getY());
//         double omegaSpeed = thetaController.calculate(currentPose.getRotation().getRadians());

//         //sets control output to the drivetrain
//         RobotContainer.drivetrain.driveWithSetpoint(xSpeed, ySpeed, omegaSpeed, true, false);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         RobotContainer.drivetrain.drive(0, 0, 0, true, false);
//     }

//     @Override
//     public boolean isFinished() {
//         return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
//     }




//     /**
//      * Find closest tag of a given type to current position of robot
//      */
//     public static int getClosestTag(Pose2d pose) {
//         Translation2d position = pose.getTranslation();

//         double closestDistance = Double.MAX_VALUE;
//         int closestId = -1;
//         for (AprilTag tag : fieldLayout.getTags()) {
//             Translation2d tagPosition = new Translation2d(tag.pose.getTranslation().getX(), tag.pose.getTranslation().getY());
//             double distance = PoseUtils.flipPositionAlliance(tagPosition).getDistance(position);
//             if (distance < closestDistance) {
//                 closestDistance = distance;
//                 closestId = tag.ID;
//             }
//         }
//         return closestId;
//     }
// }
