package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Auto;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.PoseUtils;
import frc.robot.util.TagOffset;
import frc.robot.util.Tunable;
import frc.robot.util.TunerConstants;

public class GoToPoseCommand extends Command {

    private PIDController xController;
    private PIDController yController;
    private PIDController thetaController;

    private Pose2d targetPose;
    private TagOffset direction;
 
    double targetHeading;
    // double offset = .3;
    // boolean isOffset;

    DoublePublisher xOutputPub;
    DoublePublisher yOutputPub;
    DoublePublisher rotOutputPub;

    // DoublePublisher xPosePub;
    // DoublePublisher yPosePub;
    // DoublePublisher rotPosePub;

    @Override
    public void initialize() {



        Logger.recordOutput("PoseCommand/TargetPose", targetPose);
        Logger.recordOutput("PoseCommand/TargetPose", RobotContainer.drivetrain.getRobotPose());

        // NetworkTableInstance inst = NetworkTableInstance.getDefault();
        // NetworkTable poseCommandTable = inst.getTable("Pose Command");
        //.Auto.field.setRobotPose(targetPose);

        // DoubleTopic xSetpointTopic = poseCommandTable.getDoubleTopic("X Setpoint");
        // DoublePublisher xSetpointPub = xSetpointTopic.publish();
        // DoubleTopic ySetpointTopic = poseCommandTable.getDoubleTopic("Y Setpoint");
        // DoublePublisher ySetpointPub = ySetpointTopic.publish();
        // DoubleTopic rotSetpointTopic = poseCommandTable.getDoubleTopic("Rot Setpoint");
        // DoublePublisher rotSetpointPub = rotSetpointTopic.publish();

        // xSetpointPub.set(targetPose.getX());
        // ySetpointPub.set(targetPose.getY());
        // rotSetpointPub.set(targetPose.getRotation().getRadians());

        // DoubleTopic xOutputTopic = poseCommandTable.getDoubleTopic("X Output");
        // xOutputPub = xOutputTopic.publish();
        // DoubleTopic yOutputTopic = poseCommandTable.getDoubleTopic("Y Output");
        // yOutputPub = yOutputTopic.publish();
        // DoubleTopic rotOutputTopic = poseCommandTable.getDoubleTopic("Rot Output");
        // rotOutputPub = rotOutputTopic.publish();

        // DoubleTopic xPoseTopic = poseCommandTable.getDoubleTopic("X Pose");
        // xPosePub = xPoseTopic.publish();
        // DoubleTopic yPoseTopic = poseCommandTable.getDoubleTopic("Y Pose");
        // yPosePub = yPoseTopic.publish();
        // DoubleTopic rotPoseTopic = poseCommandTable.getDoubleTopic("Rot Pose");
        // rotPosePub = rotPoseTopic.publish();
        xController.setTolerance(0.02);
        yController.setTolerance(0.02);
        xController.setSetpoint(targetPose.getX());
        yController.setSetpoint(targetPose.getY());
        thetaController.setSetpoint(targetPose.getRotation().getRadians());
    }

    public GoToPoseCommand(Pose2d targetPose) {
        addRequirements(RobotContainer.drivetrain); //requires the drivetrain
        xController = new PIDController(3.5, 0, .035); //creates the PIDControllers
        yController = new PIDController(3.5, 0, .035); //TODO tolerances
        this.targetPose = targetPose;
        thetaController = RobotContainer.drivetrain.thetaController;

        // TODO I think I need to log the 1st pose2d in disabled to prevent overruns
        Logger.recordOutput("PoseCommand/TargetPose", targetPose);
        Logger.recordOutput("PoseCommand/TargetPose", RobotContainer.drivetrain.getRobotPose());
    }

    @Override
    public void execute() {
        //current pose to PID from
        Pose2d currentPose = RobotContainer.drivetrain.getState().Pose;

        // //logging
        // xPosePub.set(currentPose.getX());
        // yPosePub.set(currentPose.getY());
        // rotPosePub.set(currentPose.getRotation().getRadians());

        //calculating outputs
        double xSpeed = xController.calculate(currentPose.getX());
        double ySpeed = yController.calculate(currentPose.getY());
        double omegaSpeed = thetaController.calculate(currentPose.getRotation().getRadians());

        //sets control output to the drivetrain
        RobotContainer.drivetrain.driveWithSetpoint(xSpeed, ySpeed, omegaSpeed, true, false);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.drivetrain.drive(0, 0, 0, true, false);
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }

}
