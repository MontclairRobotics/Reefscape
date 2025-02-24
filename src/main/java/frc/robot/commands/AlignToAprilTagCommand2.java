package frc.robot.commands;

import java.util.Optional;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
// import frc.robot.util.ScoreDirection;
import frc.robot.vision.Limelight;

// public class AlignToAprilTagCommand2 extends Command {

//     public static final double LIMELIGHT_HEIGHT = 0.1;
//     public static final double REEF_APRILTAG_HEIGHT = 0.2;
    
//     private PIDController xController;
//     private PIDController yController;
//     private PIDController thetaController;
//     private Pose2d targetPose;

//     public AlignToAprilTagCommand2(ScoreDirection direction, int tagID) {
//         xController = new PIDController(5, 0, 0);
//         yController = new PIDController(5, 0, 0);
//         thetaController = RobotContainer.drivetrain.thetaController;

//         if (direction.isLeft()) {
//             targetPose = RobotContainer.drivetrain.getClosestScoringPose(RobotContainer.drivetrain.LEFT_BLUE_SCORING_POSES);
//         } else if (direction.isRight()) {
//             targetPose = RobotContainer.drivetrain.getClosestScoringPose(RobotContainer.drivetrain.RIGHT_BLUE_SCORING_POSES);
//         } else {
//             targetPose = RobotContainer.drivetrain.getClosestScoringPose(RobotContainer.drivetrain.BLUE_SCORING_POSES);
//         }

//         addRequirements(RobotContainer.drivetrain); //requires the drivetrain
//     }

//     @Override
//     public void initialize(){
//         xController.setSetpoint(targetPose.getX());
//         yController.setSetpoint(targetPose.getY());
//         thetaController.setSetpoint(targetPose.getRotation().getRadians());
//     }

//     @Override
//     public void execute() {
//         //current pose to PID from
//         Pose2d currentPose = RobotContainer.drivetrain.getState().Pose;

//         //calculating outputs
//         double xSpeed = xController.calculate(currentPose.getX());
//         double ySpeed = yController.calculate(currentPose.getY());
//         double omegaSpeed = thetaController.calculate(currentPose.getRotation().getRadians());

//         //sets control output to the drivetrain
//         RobotContainer.drivetrain.drive(xSpeed, ySpeed, omegaSpeed, true, false);
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
//      * Trig base pose calculation
//      * @return
//      */
//     public Pose2d getCurrentPose() {
//         Optional<Double> optionalYaw = RobotContainer.limelight.getYawAtTime(RobotContainer.limelight.getTimestampSeconds());
//         if (optionalYaw.isPresent()) {
//             // TODO: Radians or degrees?
//             double yaw = optionalYaw.get();
//             double tx = RobotContainer.limelight.getTX();

//             double angleA = targetPose.getRotation().getRotations() - yaw;
//             double angleB = 90 - tx;
//             double angleC = 180 - angleA - angleB;

//             double ty = RobotContainer.limelight.getTY();
//             double lengthA = Math.tan(Math.toRadians(ty)) * (REEF_APRILTAG_HEIGHT - LIMELIGHT_HEIGHT);

//             double lengthB = lengthA / Math.sin(Math.toRadians(angleC)) * Math.sin(Math.toRadians(angleB));
//             double lengthC = lengthA / Math.sin(Math.toRadians(angleC)) * Math.sin(Math.toRadians(angleA));

//             double distance = lengthB / Math.sin(Math.toRadians(angleA)) * Math.sin(Math.toRadians(90));

//             double translationX = Math.sin(targetPose.getRotation().getRadians()) * distance;
//             double translationY = Math.cos(targetPose.getRotation().getRadians()) * distance;

//             Translation2d translation = new Translation2d(translationX, translationY); // + aprilTagPosition
//             Pose2d pose = new Pose2d(translation, targetPose.getRotation());
//             return pose;
//         }

//         return null;
//     }
// }
