package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.PoseUtils;
import frc.robot.util.TagOffset;

public class GoToPoseInputCommand extends Command{
    
    private PIDController xController;
    private PIDController yController;
    private PIDController thetaController;

    private Pose2d targetPose;
 
    double targetHeading;
    double offset = .3;
    boolean isOffset;

    // DoublePublisher xPosePub;
    // DoublePublisher yPosePub;
    // DoublePublisher rotPosePub;

    @Override
    public void initialize() {

        if(isOffset) {
            //flips the angle if we are on red, so that the trig functions will work properly
            //On red, the pose for POINT A on RED ALLIANCE has a heading of 180 (I think), 
            //but the pose for POINT A on BLUE ALLIANCE has a heading of 0 (I think), so we 
            //just have to make them the same again
            targetHeading = PoseUtils.flipRotAlliance(targetPose.getRotation()).getRadians();
            //when target heading is zero, we want the offset to be backwards but cos(0) 
            //is positive, so we multiply by negative 1
            //same thing for sin(x)
            double updatedX = targetPose.getX() + (-1 * offset * Math.cos(targetHeading));
            double updatedY = targetPose.getY() + (-1 * offset * Math.sin(targetHeading));
            //creates new updated pose
            targetPose = new Pose2d(new Translation2d(updatedX, updatedY), Rotation2d.fromRadians(targetHeading));
        }

        //Cancels the command if for some reason the target pose is null 
        //(i.e. if the direction is not CENTER, LEFT, or REVERSE)
        if(targetPose == null) {
            cancel();
        }

        xController.setSetpoint(targetPose.getX());
        yController.setSetpoint(targetPose.getY());
        thetaController.setSetpoint(targetPose.getRotation().getRadians());

    }

    public GoToPoseInputCommand(Pose2d targetPose, boolean isOffset) {
        this.targetPose = targetPose;
        addRequirements(RobotContainer.drivetrain); //requires the drivetrain
        xController = new PIDController(4, 0, .035); //creates the PIDControllers
        yController = new PIDController(4, 0, .035); //TODO tolerances
        xController.setTolerance(.01);
        yController.setTolerance(.01);
        thetaController = RobotContainer.drivetrain.thetaController;
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
