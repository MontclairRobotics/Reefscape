package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.TagOffset;
import frc.robot.vision.Limelight;

public class DistanceAlign extends Command{
    
    private PIDController xController;
    private PIDController yController;
    private PIDController thetaController;
    private TagOffset direction;

    private Limelight camera;

    public DistanceAlign(TagOffset direction) {
        this.direction = direction; //left or right for coral, center for grabbing algae
        this.camera = direction == TagOffset.LEFT ? RobotContainer.rightLimelight : RobotContainer.leftLimelight;
        xController = new PIDController(5, 0, 0);
        xController.setTolerance(0.02); //meters
        yController = new PIDController(5, 0, 0);
        yController.setTolerance(0.02); //meters
        thetaController = RobotContainer.drivetrain.thetaController;
    }

    @Override
    public void initialize(){
        xController.setSetpoint(direction.getXOffsetM());
        yController.setSetpoint(direction.getYOffsetM());
        double wrappedSetPoint = Drivetrain.wrapAngle(RobotContainer.drivetrain.odometryHeading.plus(Rotation2d.fromDegrees(camera.getTX()))).getRadians();
        thetaController.setSetpoint(wrappedSetPoint);
    }

    @Override
    public void execute() {

        //PID calculated outputs
        double xSpeed = xController.calculate(camera.getStrafeDistanceToReef());
        double ySpeed = yController.calculate(camera.getStraightDistanceToReef());
        double thetaSpeed = thetaController.calculate(RobotContainer.drivetrain.odometryHeading.getRadians());

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
        return (xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint()) || !camera.hasValidTarget();
    }

}
