package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.TagOffset;
import frc.robot.vision.Limelight;

public class AlignToAprilTagCommandOffset extends Command {
    
    private PIDController xController;
    private PIDController yController;
    private PIDController thetaController; //TODO should be PhoenixPIDController? something about timestamps?

    private Limelight camera;
    //private int tagID;
    private double xOffset;
    private double yOffset;
    private double thetaOffset;
    private double tagHeightMeters;

    public AlignToAprilTagCommandOffset(Limelight camera, double xOffset, double yOffset, double thetaOffset, double tagHeightMeters) {
        this.camera = camera;
        this.xOffset = xOffset;
        this.yOffset = yOffset;
        this.thetaOffset = thetaOffset;
        this.tagHeightMeters = tagHeightMeters;
        //TODO: tune + make global PID Constants
        xController = new PIDController(5, 0, 0);
        xController.setTolerance(0.5); //0.5 degrees, I think? if its based on tx
        yController = new PIDController(5, 0, 0);
        yController.setTolerance(0.5); //degrees
        thetaController = RobotContainer.drivetrain.thetaController;
    }

    @Override
    public void initialize(){
        //sets the tx and ty setpoints
        // TODO we want our setpoints to be zero, right?
        xController.setSetpoint(xOffset);
        yController.setSetpoint(yOffset);
        thetaController.setSetpoint(thetaOffset);
    }

    @Override
    public void execute() {
        //TODO: make these get the correct value

        //PID calculated outputs
        double xSpeed = 0;
        double ySpeed = 0;

        //TODO will this just get stuck against a wall?
        double thetaSpeed = thetaController.calculate(camera.getTX());

        if (thetaController.atSetpoint()) {
            // TODO these may need inverts
            xSpeed = xController.calculate(camera.getHorizontalDistanceToTag(tagHeightMeters));
            ySpeed = yController.calculate(camera.getStraightDistanceToTag(tagHeightMeters));
        }

        //drives robot relative because tx and ty are robot relative
        //no rotation input, we assume this is being used when robot is aligned heading-wise, but not translationally
        //can add one to also move rotationally then translate later
        //doesn't respect operator persective (this doesn't matter because its robot relative anyways)
        RobotContainer.drivetrain.drive(xSpeed, ySpeed, thetaSpeed, false, false);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.drivetrain.drive(0, 0, 0, true, false);
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint() && camera.hasValidTarget();
    }
}
