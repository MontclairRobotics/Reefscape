package frc.robot.util;

import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class PoseUtils {

    /**
     * Returns a pose flipped for the appropriate alliance
     * BE CAREFUL!!!! If this is called before the FMS finishes setting up, the
     * alliance
     * could switch after the method call, ruining things during a match (I think)
     * 
     * @param pose a pose on the blue side of the field
     * @return the same pose on the current alliance
     */
    public static Pose2d flipPoseAlliance(Pose2d pose) {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            return FlippingUtil.flipFieldPose(pose);
        }
        return pose;
    }

    public static Rotation2d flipRotAlliance(Rotation2d rot) {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            return FlippingUtil.flipFieldRotation(rot);
        }
        return rot;
    }

    public static boolean angleDeadband(Rotation2d angle1, Rotation2d angle2, Rotation2d deadband) {
        double degrees1 = wrapRotation(angle1).getDegrees();
        double degrees2 = wrapRotation(angle2).getDegrees();
        double deadbandDeg = wrapRotation(deadband).getDegrees();

        return Math.abs(degrees1 - degrees2) < deadbandDeg || Math.abs(degrees1 - degrees2) > 360 - deadbandDeg;
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
