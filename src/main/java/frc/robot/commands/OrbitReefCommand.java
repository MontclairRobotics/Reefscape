package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.PoseUtils;

public class OrbitReefCommand extends Command {

    private Pose2d targetPose = new Pose2d(4.495, 3.995, Rotation2d.kZero);
    
    private PIDController thetaController = new PIDController(RobotContainer.drivetrain.thetaController.getP(), RobotContainer.drivetrain.thetaController.getI(), RobotContainer.drivetrain.thetaController.getD());

    private PIDController radialCorrection = new PIDController(2, 0, 0);

    private double orbitRadius = 0;

    public OrbitReefCommand() {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        radialCorrection.setTolerance(0.2);
        addRequirements(RobotContainer.drivetrain);
    }

    @Override
    public void initialize() {
        targetPose = new Pose2d(PoseUtils.flipTranslationAlliance(targetPose.getTranslation()), Rotation2d.kZero);
        Logger.recordOutput("OrbitReefCommand/TargetPose", targetPose);

        Pose2d currentPose = RobotContainer.drivetrain.getRobotPose();

        Transform2d transformPose = currentPose.minus(targetPose);

        double xT = transformPose.getX();
        double yT = transformPose.getY();

        orbitRadius = Math.sqrt(xT*xT + yT*yT);
    }

    @Override
    public void execute() {


        double velocity = RobotContainer.drivetrain.getVelocityXFromController();

        Pose2d currentPose = RobotContainer.drivetrain.getRobotPose();

        Transform2d transformPose = currentPose.minus(targetPose);

        double xT = transformPose.getX();
        double yT = transformPose.getY();

        Logger.recordOutput("OrbitReefCommand/transform", transformPose);
        double currentRadius = transformPose.getTranslation().getNorm();
        // System.out.println("Orbit: " + orbitRadius + " current: " + currentRadius);

        double vy = -velocity * (transformPose.getX() / currentRadius);
        double vx = velocity * (transformPose.getY() / currentRadius); 

        System.out.println(currentRadius - orbitRadius);
        double vCorrect = radialCorrection.calculate(currentRadius, orbitRadius);
        // System.out.println(vCorrect);
        double vxCorrect = vCorrect * xT / currentRadius;
        double vyCorrect = vCorrect * yT / currentRadius;

        vx += vxCorrect;
        vy += vyCorrect;

        // angleError = Drivetrain.wrapAngle(Rotation2d.fromRadians(angleError)).getRadians();
        double vTheta = thetaController.calculate(currentPose.getRotation().getRadians(), Math.PI + Math.atan2(yT, xT));
        RobotContainer.drivetrain.drive(vx, vy, vTheta, true, false);
    }
}
