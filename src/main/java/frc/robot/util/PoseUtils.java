package frc.robot.util;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class PoseUtils {

    // TODO: replace with the actual Robocon field length
    public static final double ROBOCON_FIELD_LENGTH = RobotContainer.drivetrain.field.getFieldLength(); // example
    public static final double ROBOCON_FIELD_WIDTH = RobotContainer.drivetrain.field.getFieldWidth();
    /**
     * Returns a pose flipped for the appropriate alliance based on Robocon field
     * center line.
     * 
     * @param pose a pose on the blue side of the field
     * @return the same pose on the current alliance
     */
    public static Pose2d flipPoseAlliance(Pose2d pose) {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            return flipPoseRobocon(pose);
        }
        return pose;
    }

    public static Translation2d flipTranslationAlliance(Translation2d trans) {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            return flipTranslationRobocon(trans);
        }
        return trans;
    }

    public static Rotation2d flipRotAlliance(Rotation2d rot) {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            return flipRotationRobocon(rot);
        }
        return rot;
    }

    // ---------------- Robocon-specific flipping ----------------

    public static Pose2d flipPoseRobocon(Pose2d pose) {
        return new Pose2d(flipTranslationRobocon(pose.getTranslation()),
         flipRotationRobocon(pose.getRotation()));
    }

    public static Translation2d flipTranslationRobocon(Translation2d trans) {
        double flippedX = ROBOCON_FIELD_LENGTH - trans.getX();
        double flippedY = ROBOCON_FIELD_WIDTH - trans.getY();
        return new Translation2d(flippedX, flippedY);
    }

    public static Rotation2d flipRotationRobocon(Rotation2d rot) {
        // Mirror rotation horizontally
        return FlippingUtil.flipFieldRotation(rot);
    }

    // ---------------- Angle utilities (unchanged) ----------------

    public static boolean angleDeadband(Rotation2d angle1, Rotation2d angle2, Rotation2d deadband) {
        double degrees1 = wrapRotation(angle1).getDegrees();
        double degrees2 = wrapRotation(angle2).getDegrees();
        double deadbandDeg = wrapRotation(deadband).getDegrees();

        return Math.abs(degrees1 - degrees2) < deadbandDeg
                || Math.abs(degrees1 - degrees2) > 360 - deadbandDeg;
    }

    public static Rotation2d wrapRotation(Rotation2d rot) {
        double degrees = rot.getDegrees() % 360;
        if (degrees < 0) {
            degrees += 360;
        }
        return Rotation2d.fromDegrees(degrees);
    }

    public static Rotation2d getAngleDistance(Rotation2d rot1, Rotation2d rot2) {
        rot1 = wrapRotation(rot1);
        rot2 = wrapRotation(rot2);
        double ang1 = rot1.getDegrees();
        double ang2 = rot2.getDegrees();
        double distance = Math.min(Math.abs(ang1 - ang2), 360 - Math.abs(ang1 - ang2));
        return Rotation2d.fromDegrees(distance);
    }
}