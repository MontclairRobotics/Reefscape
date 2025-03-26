package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.PoseUtils;

public class FaceReefCommand extends Command {

    private Pose2d targetPose = new Pose2d(4.495, 3.995, Rotation2d.kZero);
    
    private PIDController thetaController = new PIDController(RobotContainer.drivetrain.thetaController.getP(), RobotContainer.drivetrain.thetaController.getI(), RobotContainer.drivetrain.thetaController.getD());


    public FaceReefCommand() {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(RobotContainer.drivetrain);
    }

    @Override
    public void initialize() {
        targetPose = new Pose2d(PoseUtils.flipTranslationAlliance(targetPose.getTranslation()), Rotation2d.kZero);
        Logger.recordOutput("FaceReefCommand/TargetPose", targetPose);
    }

    @Override
    public void execute() {


        double vy = RobotContainer.drivetrain.getVelocityXFromController();
        double vx = RobotContainer.drivetrain.getVelocityYFromController();

        Pose2d currentPose = RobotContainer.drivetrain.getRobotPose();

        Transform2d transformPose = currentPose.minus(targetPose);

        double xT = transformPose.getX();
        double yT = transformPose.getY();

        Logger.recordOutput("OrbitReefCommand/transform", transformPose);

        double vTheta = thetaController.calculate(currentPose.getRotation().getRadians(), Math.PI + Math.atan2(yT, xT));
        RobotContainer.drivetrain.drive(vx, vy, vTheta, true, true);
    }
}
