package frc.robot.subsystems.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import static frc.robot.subsystems.Drive.DriveState.*;

public class DriveState extends SubsystemBase {
    
    /* CONSTANTS ------------------------------------------------------------------------------------------- */ 

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    public static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    public static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    public static boolean m_hasAppliedOperatorPerspective = false;
    
    /* variable to store our heading  */
    public static Rotation2d odometryHeading;

    public static boolean isRobotAtAngleSetPoint;
    public static boolean isFieldRelative;

    public DriveState() {}
    /**
     * @return the odometry heading, in rotations
     */
    public static Rotation2d getYawRotations() {
        return odometryHeading;
    }
    
    /**
     * @return A command to change to ROBOT relative driving
     */
    public static Command toRobotRelative() {
        return Commands.runOnce(() -> {
            isFieldRelative = false;
        }, RobotContainer.drivetrain);
      }

    /**
     * @return A command to change to FIELD relative driving
     */
    public static Command toFieldRelative (){
        return Commands.runOnce(() -> {
            isFieldRelative = true;
        }, RobotContainer.drivetrain);
    }

    /**
     * Zeroes the Pigeon 2 IMU (our gyroscope)
     */
    public static void zeroGyro(){
        RobotContainer.drivetrain.getPigeon2().setYaw(0);
    }

    /**
     * @return A command to zero the drivetrain
     */
    public static Command zeroGyroCommand() {
        return Commands.runOnce(() -> zeroGyro(), RobotContainer.drivetrain);
    } 

    /** 
     * wraps the given angle from -180 to 180 degress
     * @param ang the angle to wrap, in rotations
     * @return the wrapped angle, in rotations
    */
    public static Rotation2d wrapAngle(Rotation2d ang) {
        double angle = ang.getDegrees();
        angle = (angle + 180) % 360; // Step 1 and 2
        if (angle < 0) {
            angle += 360; // Make sure it's positive
        }
        return Rotation2d.fromDegrees(angle - 180); // Step 3
    }
    
    /**
     * 
     * @return the odometry heading, wrapped between -1/2 rotations to 1/2 rotations.
     */
    public static Rotation2d getWrappedHeading() {
        return wrapAngle(getYawRotations());
    }

    /**
     * @return the current robot pose
     */
    public static Pose2d getRobotPose() {
        return RobotContainer.drivetrain.getState().Pose;
    }

    /**
     * @return the current ChassisSpeeds of the drivetrain
     */
    public static ChassisSpeeds getCurrentSpeeds() {
        return RobotContainer.drivetrain.getState().Speeds;
    }

    @Override
    public void periodic() {
        isFieldRelative = !RobotContainer.driverController.L2().getAsBoolean(); //Could be improved later, might cause overruns
        isRobotAtAngleSetPoint = RobotContainer.drivetrain.thetaController.atSetpoint();
        odometryHeading = RobotContainer.drivetrain.getState().Pose.getRotation();
    }
}
