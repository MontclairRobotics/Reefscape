package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.Drive.DriveState;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.TunerConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive.Drivetrain;

public class GoToPoseCommandContinuous extends Command {

    private ProfiledPIDController xController;
    private ProfiledPIDController yController;
    private ProfiledPIDController thetaController;

    private Supplier<Pose2d> poseSupplier;
    
    public GoToPoseCommandContinuous(Supplier<Pose2d> pose) {
        addRequirements(RobotContainer.drivetrain);
        xController = new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond), Drivetrain.FORWARD_ACCEL_LIMIT));
        yController = new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond), Drivetrain.SIDE_ACCEL_LIMIT));
        thetaController = new ProfiledPIDController(RobotContainer.drivetrain.thetaController.getP(), RobotContainer.drivetrain.thetaController.getI(), RobotContainer.drivetrain.thetaController.getD(), new TrapezoidProfile.Constraints(Drivetrain.MAX_ROT_SPEED, Drivetrain.ROT_ACCEL_LIMIT));
        thetaController.enableContinuousInput(-180, 180);
        poseSupplier = pose;
    }

    @Override
    public void execute() {
        xController.setGoal(poseSupplier.get().getX());
        yController.setGoal(poseSupplier.get().getY());
        thetaController.setGoal(poseSupplier.get().getRotation().getRadians());
        ChassisSpeeds speeds = RobotContainer.drivetrain.getDriveState().getCurrentSpeeds();
        double xSpeed = xController.calculate(speeds.vxMetersPerSecond);
        double ySpeed = yController.calculate(speeds.vyMetersPerSecond);
        double omegaSpeed = thetaController.calculate(speeds.omegaRadiansPerSecond);

        RobotContainer.drivetrain.drive(xSpeed, ySpeed, omegaSpeed, true);        
        
    }

    @Override
    public void end(boolean interrupted) {
        //Needed?
        RobotContainer.drivetrain.drive(0, 0, 0, true);

    }

    @Override
    public boolean isFinished() {
        //.atSetpoint?
        return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
    }
    
}
