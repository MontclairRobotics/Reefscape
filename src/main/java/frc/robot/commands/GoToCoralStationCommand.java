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
    private boolean isOffset;
    boolean isLeft;
    int rightStation = 0;
    int leftStation = 1;
    double targetHeading;
    double otherOffset = .2;

    public void initialize() {
        targetPose = new Pose2d();
        if(isLeft) {
            targetPose = Drivetrain.BLUE_INTAKE_POSES[leftStation];
        } else {
            targetPose = Drivetrain.BLUE_INTAKE_POSES[rightStation];
        }


        if(targetPose == null) {
            cancel();
        }
      

        xController.setTolerance(0.02);
        yController.setTolerance(0.02);
        xController.setSetpoint(targetPose.getX());
        yController.setSetpoint(targetPose.getY());
        thetaController.setSetpoint(targetPose.getRotation().getRadians());
    }

    public GoToCoralStationCommand(TagOffset direction, boolean isLeft, boolean isOffset) {
        this.direction = direction; //sets the direction
        this.isLeft = isLeft;
        this.isOffset = isOffset;
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
