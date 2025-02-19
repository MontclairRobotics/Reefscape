package frc.robot.commands;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.TagOffset;
import frc.robot.vision.Limelight;

public class AlignToReefTagCommand extends Command {
    
    private PIDController xController;
    private PIDController yController;
    private PIDController thetaController;
    //private int tagID;
    private TagOffset direction;

    private Limelight camera;

    public AlignToReefTagCommand(TagOffset direction, Limelight camera) {
        this.direction = direction; //left or right for coral, center for grabbing algae
        this.camera = camera;
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
        //TODO: does PIDing to a tx setpoint actually work?
        xController.setSetpoint(direction.getTxTargetError());
        yController.setSetpoint(direction.getTyTargetError());
    }

    @Override
    public void execute() {

        //PID calculated outputs
        double xSpeed = xController.calculate(camera.getTX());
        double ySpeed = yController.calculate(camera.getTY());

        //drives robot relative because tx and ty are robot relative
        //no rotation input, we assume this is being used when robot is aligned heading-wise, but not translationally
        //can add one to also move rotationally then translate later
        //doesn't respect operator persective (this doesn't matter because its robot relative anyways)
        RobotContainer.drivetrain.drive(xSpeed, ySpeed, 0, false, false);
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
