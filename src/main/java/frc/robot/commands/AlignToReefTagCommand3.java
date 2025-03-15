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

public class AlignToReefTagCommand3 extends Command {
    
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
    
    public AlignToReefTagCommand3(int tagId, TagOffset direction, Limelight camera) {
        this.tagId = tagId;
        this.direction = direction; //left or right for coral, center for grabbing algae
        this.camera = camera;
        //TODO: tune + make global PID Constants
        xController = new PIDController(.1, 0, 0);
        xController.setTolerance(0.5); //0.5 degrees, I think? if its based on tx
        yController = new PIDController(0.5, 0, 0);
        yController.setTolerance(0.5); //degrees
        thetaController = RobotContainer.drivetrain.thetaController;
    }

    @Override
    public void initialize(){
        camera.setPriorityTagID(tagId);

        tagRotation = tagRotationsMap.get(tagId).getRadians();
        thetaController.setSetpoint(direction.getTxTargetError());

        //sets the tx and ty setpoints
        //TODO: does PIDing to a tx setpoint actually work?
        xController.setSetpoint(direction.getTxTargetError());
        yController.setSetpoint(direction.getTyTargetError());

        // Set last value to ideal value to start, but will get updated immediately
        lastTx = direction.getTxTargetError();
        lastTy = direction.getTyTargetError();
    }

    @Override
    public void execute() {

        // Use last known values if tag isn't visible right now
        double tx = lastTx;
        double ty = lastTy;
        double theta = RobotContainer.drivetrain.getRobotPose().getRotation().getRadians();
        double thetaDiff = Units.radiansToDegrees(theta - tagRotation);
        Logger.recordOutput("Align/thetaDiff", thetaDiff);

        
        if (camera.getTagID() == tagId) {
            tx = camera.getTX(); 
            ty = camera.getTY();

            lastTx = tx;
            lastTy = ty;
        }
        Logger.recordOutput("Align/TX", tx);
        Logger.recordOutput("Align/TXThetaDiff", tx - thetaDiff);
        Logger.recordOutput("Align/TY", ty);

        // double p = (.2 - Math.sin(Units.degreesToRadians(ty))) / 2; // 0.01 + Math.pow(0.4, ty);
        // Logger.recordOutput("Align/p", p);
        // xController.setP(p);

        

        thetaController.setSetpoint(Units.degreesToRadians(0));
        Logger.recordOutput("Align/thetaSetpoint1", 0.0);
        double thetaSpeed1 = thetaController.calculate(Units.degreesToRadians(tx));
        Logger.recordOutput("Align/thetaSpeed1", thetaSpeed1);

        thetaController.setSetpoint(tagRotation);
        Logger.recordOutput("Align/thetaSetpoint2", tagRotation);
        double thetaSpeed2 = thetaController.calculate(theta);
        Logger.recordOutput("Align/thetaSpeed2", thetaSpeed2);

        double ratio = Math.min(ty / 4, 1);
        Logger.recordOutput("Align/ratio", ratio);
        double thetaSpeed = ratio * thetaSpeed2 + (1 - ratio) * thetaSpeed1;
        
        double distance = (REEF_APRIL_TAG_HEIGHT - camera.getCameraHeightMeters()) / Math.tan(Units.degreesToRadians(ty));
        Logger.recordOutput("Align/distance", distance);
        double xSpeed1 = Math.sin(thetaDiff) * distance/2;
        double xSpeed2 = xController.calculate(tx);
        double xSpeed = ratio * xSpeed2 + (1 - ratio) * xSpeed1;

        double ySpeed = yController.calculate(ty);
        Logger.recordOutput("Align/thetaSpeed", thetaSpeed);
        Logger.recordOutput("Align/xSpeed", xSpeed);
        Logger.recordOutput("Align/ySpeed", ySpeed);

        // double fieldRelativeXSpeed = xSpeed * Math.sin(Units.degreesToRadians(tagRotation)) + ySpeed * Math.cos(Units.degreesToRadians(tagRotation));
        // double fieldRelativeYSpeed = xSpeed * Math.cos(Units.degreesToRadians(tagRotation)) - ySpeed * Math.sin(Units.degreesToRadians(tagRotation));
        // Logger.recordOutput("Align/fieldRelativeXSpeed", fieldRelativeXSpeed);
        // Logger.recordOutput("Align/fieldRelativeYSpeed", fieldRelativeYSpeed);

        //drives robot relative because tx and ty are robot relative
        //no rotation input, we assume this is being used when robot is aligned heading-wise, but not translationally
        //can add one to also move rotationally then translate later
        //doesn't respect operator persective (this doesn't matter because its robot relative anyways)
        RobotContainer.drivetrain.drive(ySpeed, xSpeed, thetaSpeed, false, false);
        // RobotContainer.drivetrain.drive(fieldRelativeXSpeed, fieldRelativeYSpeed, thetaSpeed, true, false);
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
