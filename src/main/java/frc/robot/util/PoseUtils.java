package frc.robot.util;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class PoseUtils {

    /**
     * Returns a pose flipped for the appropriate alliance
     * BE CAREFUL!!!! If this is called before the FMS finishes setting up, the alliance
     * could switch after the method call, ruining things during a match (I think)
     * @param pose a pose on the blue side of the field
     * @return the same pose on the current alliance
     */
    public static Pose2d flipPoseAlliance(Pose2d pose) {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            return FlippingUtil.flipFieldPose(pose);
        }
        return pose;
    }
}
