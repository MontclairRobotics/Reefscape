package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Auto;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.TunerConstants;

public class GoToPoseCommand extends Command {

    private PIDController xController;
    private PIDController yController;
    private PIDController thetaController;

    private Pose2d targetPose;

    DoublePublisher xOutputPub;
    DoublePublisher yOutputPub;
    DoublePublisher rotOutputPub;

    DoublePublisher xPosePub;
    DoublePublisher yPosePub;
    DoublePublisher rotPosePub;

    @Override
    public void initialize() {


        targetPose = RobotContainer.drivetrain.getClosestScoringPose(Drivetrain.BLUE_SCORING_POSES);
        
        if(RobotContainer.driverController.L1().getAsBoolean()) {
            targetPose = RobotContainer.drivetrain.getClosestScoringPose(Drivetrain.LEFT_BLUE_SCORING_POSES);
        } else if(RobotContainer.driverController.R1().getAsBoolean()) {
            targetPose = RobotContainer.drivetrain.getClosestScoringPose(Drivetrain.RIGHT_BLUE_SCORING_POSES);
        } 
        
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable poseCommandTable = inst.getTable("Pose Command");
        //.Auto.field.setRobotPose(targetPose);

    
        DoubleTopic xSetpointTopic = poseCommandTable.getDoubleTopic("X Setpoint");
        DoublePublisher xSetpointPub = xSetpointTopic.publish();
        DoubleTopic ySetpointTopic = poseCommandTable.getDoubleTopic("Y Setpoint");
        DoublePublisher ySetpointPub = ySetpointTopic.publish();
        DoubleTopic rotSetpointTopic = poseCommandTable.getDoubleTopic("Rot Setpoint");
        DoublePublisher rotSetpointPub = rotSetpointTopic.publish();

        xSetpointPub.set(targetPose.getX());
        ySetpointPub.set(targetPose.getY());
        rotSetpointPub.set(targetPose.getRotation().getRadians());

        DoubleTopic xOutputTopic = poseCommandTable.getDoubleTopic("X Output");
        xOutputPub = xOutputTopic.publish();
        DoubleTopic yOutputTopic = poseCommandTable.getDoubleTopic("Y Output");
        yOutputPub = yOutputTopic.publish();
        DoubleTopic rotOutputTopic = poseCommandTable.getDoubleTopic("Rot Output");
        rotOutputPub = rotOutputTopic.publish();

        DoubleTopic xPoseTopic = poseCommandTable.getDoubleTopic("X Pose");
        xPosePub = xPoseTopic.publish();
        DoubleTopic yPoseTopic = poseCommandTable.getDoubleTopic("Y Pose");
        yPosePub = yPoseTopic.publish();
        DoubleTopic rotPoseTopic = poseCommandTable.getDoubleTopic("Rot Pose");
        rotPosePub = rotPoseTopic.publish();
        
        xController.setSetpoint(targetPose.getX());
        yController.setSetpoint(targetPose.getY());
        thetaController.setSetpoint(targetPose.getRotation().getRadians());
    }

    public GoToPoseCommand() {
        addRequirements(RobotContainer.drivetrain);
        xController = new PIDController(5, 0, .3);
        yController = new PIDController(5, 0, .3);
        thetaController = new PIDController(RobotContainer.drivetrain.thetaController.getP(),
                RobotContainer.drivetrain.thetaController.getI(), RobotContainer.drivetrain.thetaController.getD());
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        //targetPose = RobotContainer.drivetrain.getClosestScoringPose();
        System.out.println("Target Pose" + targetPose);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = RobotContainer.drivetrain.getCurrentSpeeds();
        Pose2d currentPose = RobotContainer.drivetrain.getState().Pose;

        xPosePub.set(currentPose.getX());
        yPosePub.set(currentPose.getY());
        rotPosePub.set(currentPose.getRotation().getRadians());

        double xSpeed = xController.calculate(currentPose.getX());
        double ySpeed = yController.calculate(currentPose.getY());
        double omegaSpeed = thetaController.calculate(currentPose.getRotation().getRadians());

        RobotContainer.drivetrain.drive(xSpeed, ySpeed, omegaSpeed, true, false);

    }

    @Override
    public void end(boolean interrupted) {
        // Needed?
        RobotContainer.drivetrain.drive(0, 0, 0, true, false);

    }

    @Override
    public boolean isFinished() {
        // .atSetpoint?
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }

}
