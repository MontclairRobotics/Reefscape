package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.TagOffset;
import frc.robot.vision.Limelight;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.LimelightHelpers.RawFiducial;

public class AlignToClosestReefTagOffset extends Command {
    
    private PIDController xController;
    private PIDController yController;
    private PIDController thetaController;

    private Limelight camera;
    //private int tagID;
    private static double xOffset;
    private static double yOffset;
    private double thetaOffset;


    public AlignToClosestReefTagOffset(TagOffset offset, boolean isOffset) {
        if (offset.isLeft()) {
            camera = RobotContainer.leftLimelight;
        } else {
            camera = RobotContainer.rightLimelight;
        }

        xOffset = offset.getXOffsetM();
        if (isOffset) {
            yOffset = 0.3;
        } else {
            yOffset = 0.02;
        }

        //TODO: tune + make global PID Constants
        xController = new PIDController(3.5, 0, 0);
        xController.setTolerance(0.02); //0.5 degrees, I think? if its based on tx
        yController = new PIDController(3.5, 0, 0);
        yController.setTolerance(0.02); //degrees
        thetaController = RobotContainer.drivetrain.thetaController;
        addRequirements(RobotContainer.drivetrain);
    }

    @Override
    public void initialize(){
        //sets the tx and ty setpoints
        // TODO we want our setpoints to be zero, right?

        xController.setSetpoint(xOffset);
        yController.setSetpoint(yOffset);
        thetaController.setSetpoint(thetaOffset * (Math.PI / 180.0));
    }

    @Override
    public void execute() {
        //TODO: make these get the correct value

        //PID calculated outputs
        double xSpeed = 0;
        double ySpeed = 0;

        RawFiducial closest = camera.getClosestTag();
        //TODO will this just get stuck against a wall?
        double thetaSpeed = thetaController.calculate(RobotContainer.drivetrain.getRobotPose().getRotation().getRadians());

        // if (thetaController.atSetpoint()) {

            double xDist = closest.distToCamera * -Math.sin(closest.txnc * (Math.PI / 180.0)) + camera.cameraOffsetX; // TODO invert?
            double yDist = closest.distToCamera * -Math.cos(closest.txnc * (Math.PI / 180.0)) + camera.cameraOffsetY;
            // TODO these may need inverts
            xSpeed = xController.calculate(xDist);
            ySpeed = yController.calculate(yDist);
        // }

        //drives robot relative because tx and ty are robot relative
        //no rotation input, we assume this is being used when robot is aligned heading-wise, but not translationally
        //can add one to also move rotationally then translate later
        //doesn't respect operator persective (this doesn't matter because its robot relative anyways)
        RobotContainer.drivetrain.driveWithSetpoint(xSpeed, ySpeed, thetaSpeed, false, false, false);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.drivetrain.drive(0, 0, 0, true, false);
    }

    @Override
    public boolean isFinished() {
        return (xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint()) || !camera.hasValidTarget();
    }
}
