package frc.robot.commands;

import java.util.HashMap;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.TagOffset;
import frc.robot.vision.Limelight;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.LimelightHelpers.RawFiducial;

public class AlignToReefTagCommand4 extends Command {
    
    public static double REEF_APRIL_TAG_HEIGHT = Units.inchesToMeters(12);

    private PIDController xController;
    private PIDController yController;
    private PIDController thetaController;

    private int tagId;
    private TagOffset direction;
    private double tagRotation;

    private double lastTx = 0;
    private double lastTy = 0;

    private Limelight camera;

    public static HashMap<Integer, Rotation2d> tagRotationsMap = new HashMap<Integer, Rotation2d>();
    {
        tagRotationsMap.put(6, Rotation2d.fromDegrees(120));
        tagRotationsMap.put(7, Rotation2d.fromDegrees(180));
        tagRotationsMap.put(8, Rotation2d.fromDegrees(-120));
        tagRotationsMap.put(9, Rotation2d.fromDegrees(-60));
        tagRotationsMap.put(10, Rotation2d.fromDegrees(0));
        tagRotationsMap.put(11, Rotation2d.fromDegrees(60));

        // TODO: Should these be flipped?
        tagRotationsMap.put(17, Rotation2d.fromDegrees(60));
        tagRotationsMap.put(18, Rotation2d.fromDegrees(0));
        tagRotationsMap.put(19, Rotation2d.fromDegrees(-60));
        tagRotationsMap.put(20, Rotation2d.fromDegrees(-120));
        tagRotationsMap.put(21, Rotation2d.fromDegrees(180));
        tagRotationsMap.put(22, Rotation2d.fromDegrees(120));
    }
    
    public AlignToReefTagCommand4(int tagId, TagOffset direction, Limelight camera) {
        this.tagId = tagId;
        this.direction = direction; //left or right for coral, center for grabbing algae
        this.camera = camera;
        //TODO: tune + make global PID Constants
        xController = new PIDController(2.5, 0, 0);
        xController.setTolerance(0.5); //0.5 degrees, I think? if its based on tx
        yController = new PIDController(0.5, 0, 0);
        yController.setTolerance(0.5); //degrees
        thetaController = RobotContainer.drivetrain.thetaController;
    }

    @Override
    public void initialize(){
        camera.setPriorityTagID(tagId);

        tagRotation = tagRotationsMap.get(tagId).getDegrees();
        thetaController.setSetpoint(tagRotation);
        Logger.recordOutput("Align/tagRotation", tagRotation);

        double targetTX = direction.getTxTargetError();
        double targetTY = direction.getTyTargetError();
        Logger.recordOutput("Align/targetTX", targetTX);
        Logger.recordOutput("Align/targetTY", targetTY);

        //sets the tx and ty setpoints
        //TODO: does PIDing to a tx setpoint actually work?
        double approxTargetDistance = (camera.getCameraHeightMeters() - REEF_APRIL_TAG_HEIGHT) / Math.tan(Units.degreesToRadians(targetTY + camera.getCameraAngle()));
        double targetX = approxTargetDistance * Math.sin(Units.degreesToRadians(targetTX));
        Logger.recordOutput("Align/approxTargetDistance", approxTargetDistance);
        Logger.recordOutput("Align/targetX", targetX);

        xController.setSetpoint(targetX);
        yController.setSetpoint(targetTY);

        // Set last value to ideal value to start, but will get updated immediately
        lastTx = targetTX;
        lastTy = targetTY;
    }

    @Override
    public void execute() {

        
        double tx, ty;
        double theta = RobotContainer.drivetrain.getRobotPose().getRotation().getDegrees();
        double thetaDiff = theta - tagRotation;
        Logger.recordOutput("Align/theta", theta);
        Logger.recordOutput("Align/thetaDiff", thetaDiff);

        // Use
        if (camera.getTagID() == tagId) {
            tx = camera.getTX(); 
            ty = camera.getTY();

            lastTx = tx;
            lastTy = ty;
        } else {
            // Use last known values if tag isn't visible right now
            tx = lastTx;
            ty = lastTy;
        }
        Logger.recordOutput("Align/TX", tx);
        Logger.recordOutput("Align/TY", ty);
        Logger.recordOutput("Align/TXThetaDiff", tx - thetaDiff);

        double approxYDistance = (camera.getCameraHeightMeters() - REEF_APRIL_TAG_HEIGHT) / Math.tan(Units.degreesToRadians(ty + camera.getCameraAngle()));
        double approxXDistance = approxYDistance * Math.tan(Units.degreesToRadians(thetaDiff));
        double approxDistance = Math.sqrt(approxXDistance * approxXDistance + approxYDistance * approxYDistance);
        double x = approxDistance * Math.sin(Units.degreesToRadians(tx));
        // double p = Math.max(0.02, .04 * approxDistance);
        // xController.setP(p);
        Logger.recordOutput("Align/approxXDistance", approxXDistance);
        Logger.recordOutput("Align/approxYDistance", approxYDistance);
        Logger.recordOutput("Align/approxDistance", approxDistance);
        Logger.recordOutput("Align/x", x);
        // Logger.recordOutput("Align/p", p);

        // double p = (.2 - Math.sin(Units.degreesToRadians(ty))) / 2; // 0.01 + Math.pow(0.4, ty);
        // 
        // 
        double targetTY = direction.getTyTargetError();
        double approxTargetDistance = (camera.getCameraHeightMeters() - REEF_APRIL_TAG_HEIGHT) / Math.tan(Units.degreesToRadians(targetTY + camera.getCameraAngle()));
        double ratio = (approxDistance - approxTargetDistance) / 4;
        double thetaSetpoint = ratio * tx + (1 - ratio) * tagRotation;
        Logger.recordOutput("Align/ratio", ratio);
        Logger.recordOutput("Align/thetaSetpoint", thetaSetpoint);

        thetaController.setSetpoint(thetaSetpoint);
        double thetaSpeed = thetaController.calculate(theta);
        Logger.recordOutput("Align/thetaSpeed", thetaSpeed);
    
        double xSpeed = xController.calculate(x);
        double ySpeed = yController.calculate(ty);
        Logger.recordOutput("Align/xSpeed", xSpeed);
        Logger.recordOutput("Align/ySpeed", ySpeed);

        double fieldRelativeXSpeed = ySpeed * Math.cos(Units.degreesToRadians(tx));
        double fieldRelativeYSpeed = -ySpeed * Math.sin(Units.degreesToRadians(tx));
        Logger.recordOutput("Align/fieldRelativeXSpeed", fieldRelativeXSpeed);
        Logger.recordOutput("Align/fieldRelativeYSpeed", fieldRelativeYSpeed);

        //drives robot relative because tx and ty are robot relative
        //no rotation input, we assume this is being used when robot is aligned heading-wise, but not translationally
        //can add one to also move rotationally then translate later
        //doesn't respect operator persective (this doesn't matter because its robot relative anyways)
        // RobotContainer.drivetrain.drive(ySpeed, xSpeed, thetaSpeed, false, false);
        RobotContainer.drivetrain.drive(fieldRelativeXSpeed, fieldRelativeYSpeed, thetaSpeed, true, false);
        // RobotContainer.drivetrain.drive(0, 0, thetaSpeed, true, false);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.drivetrain.drive(0, 0, 0, true, false);
    }

    @Override
    public boolean isFinished() {
        return (xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint());
    }
}
