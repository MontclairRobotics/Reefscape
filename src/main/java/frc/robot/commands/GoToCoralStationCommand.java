package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.TagOffset;

public class GoToCoralStationCommand extends Command {
    
    private PIDController xController;
    private PIDController yController;
    private PIDController thetaController;
    private Pose2d targetPose;
    private TagOffset direction;
    double targetHeading;
    double otherOffset = .05;

    public void initialize() {
        targetPose = new Pose2d();
        if(direction.isLeft()) {
            targetPose = RobotContainer.drivetrain.getClosestTargetPose(Drivetrain.LEFT_BLUE_INTAKE_POSES);
        } else if(direction.isRight()) {
            targetPose = RobotContainer.drivetrain.getClosestTargetPose(Drivetrain.RIGHT_BLUE_INTAKE_POSES);
        } else {
            targetPose = RobotContainer.drivetrain.getClosestTargetPose(Drivetrain.BLUE_INTAKE_POSES);
        }
        if(targetPose == null) {
            cancel();
        }
        //flips the angle if we are on red, so that the trig functions will work properly
            //On red, the pose for POINT A on RED ALLIANCE has a heading of 180 (I think), 
            //but the pose for POINT A on BLUE ALLIANCE has a heading of 0 (I think), so we 
            //just have to make them the same again
            targetHeading = targetPose.getRotation().getRadians();
            //when target heading is zero, we want the offset to be backwards but cos(0) 
            //is positive, so we multiply by negative 1
            //same thing for sin(x)
            double updatedX = targetPose.getX() + (-1 * otherOffset * Math.cos(targetHeading));
            double updatedY = targetPose.getY() + (-1 * otherOffset * Math.sin(targetHeading));
            //creates new updated pose
            targetPose = new Pose2d(new Translation2d(updatedX, updatedY), Rotation2d.fromRadians(targetHeading));

        xController.setTolerance(0.02);
        yController.setTolerance(0.02);
        xController.setSetpoint(targetPose.getX());
        yController.setSetpoint(targetPose.getY());
        thetaController.setSetpoint(targetPose.getRotation().getRadians());
    }

    public GoToCoralStationCommand(TagOffset direction) {
        this.direction = direction; //sets the direction
        addRequirements(RobotContainer.drivetrain); //requires the drivetrain
        xController = new PIDController(3.5, 0, .035); //creates the PIDControllers
        yController = new PIDController(3.5, 0, .035); //TODO tolerances
        thetaController = RobotContainer.drivetrain.thetaController;
    }

    public void execute() {
        //current pose to PID from
        Pose2d currentPose = RobotContainer.drivetrain.getState().Pose;

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
