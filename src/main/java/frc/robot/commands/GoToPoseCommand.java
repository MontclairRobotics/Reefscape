package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConfig;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;

public class GoToPoseCommand extends Command {

    private ProfiledPIDController xController;
    private ProfiledPIDController yController;
    private ProfiledPIDController thetaController;

    private Pose2d targetPose;
    
    @Override
    public void initialize() {
        xController.setGoal(targetPose.getX());
        yController.setGoal(targetPose.getY());
        thetaController.setGoal(targetPose.getRotation().getRadians());
    }

    public GoToPoseCommand(Pose2d targePose) {
        addRequirements(RobotContainer.drivetrain);
        xController = new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(DriveConfig.kSpeedAt12Volts.in(Units.MetersPerSecond), DriveConstants.FORWARD_ACCEL));
        yController = new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(DriveConfig.kSpeedAt12Volts.in(Units.MetersPerSecond), DriveConstants.SIDE_ACCEL));
        thetaController = new ProfiledPIDController(RobotContainer.drivetrain.thetaController.getP(), RobotContainer.drivetrain.thetaController.getI(), RobotContainer.drivetrain.thetaController.getD(), new TrapezoidProfile.Constraints(DriveConstants.MAX_ROT_SPEED, DriveConstants.ROT_ACCEL));
        thetaController.enableContinuousInput(-180, 180);
        this.targetPose = targetPose;
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = RobotContainer.drivetrain.getCurrentSpeeds();
        double xSpeed = xController.calculate(speeds.vxMetersPerSecond);
        double ySpeed = yController.calculate(speeds.vyMetersPerSecond);
        double omegaSpeed = thetaController.calculate(speeds.omegaRadiansPerSecond);

        RobotContainer.drivetrain.swerveDrive.setControl(new SwerveRequest.FieldCentric());
        
        
    }

    @Override
    public void end(boolean interrupted) {


    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
