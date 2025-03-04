package frc.robot.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;

public class ElevatorLimelight extends Limelight {

    public ElevatorLimelight(String cameraName, double cameraHeightMeters, double cameraAngle, double cameraOffsetX,
            double cameraOffsetY, boolean cameraUpsideDown) {
        super(cameraName, cameraHeightMeters, cameraAngle, cameraOffsetX, cameraOffsetY, cameraUpsideDown);
    }

    //TODO do we even want poses from this LL? maybe only if robot is still? if elevator down?
    @Override
    public void poseEstimationMegatag2() {

        // TODO does the angle need to be wrapped between 0 and 360
        double angle = (RobotContainer.drivetrain.getWrappedHeading().getDegrees() + 360) % 360;
        LimelightHelpers.SetRobotOrientation(cameraName, angle, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

        boolean shouldRejectUpdate = false;
        if (mt2 != null) {
            if (mt2.tagCount == 0) {
                // rejects current measurement if there are no aprilTags
                shouldRejectUpdate = true;
            }
            if (Math.abs(RobotContainer.drivetrain.getCurrentSpeeds().omegaRadiansPerSecond) > angleVelocityTolerance) {
                shouldRejectUpdate = true;
            }
            if (RobotContainer.elevator.isVelociatated()) {
                shouldRejectUpdate = true;
            }
            // adds vision measurement if conditions are met
            if (!shouldRejectUpdate) {
                Logger.recordOutput(cameraName + "/mt2Pose", mt2.pose);
                // RobotContainer.drivetrain.swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
                Matrix<N3, N1> stdDevs = VecBuilder.fill(.7,.7,9999999);
                RobotContainer.drivetrain.addVisionMeasurement(
                        mt2.pose,
                        mt2.timestampSeconds);
            }
        }
    }

    double bottomOfCameraToLense = 0.016107; ///TODO get right number
    @Override
    public void periodic() {
        // tagID = (int) Limetable.getEntry("tid").getDouble(-1);
        Pose3d pos = LimelightHelpers.getCameraPose3d_RobotSpace(cameraName);
        LimelightHelpers.setCameraPose_RobotSpace(cameraName, pos.getY(), pos.getX(), RobotContainer.elevator.getHeight() + bottomOfCameraToLense, cameraAngle, 0, 0);
        super.periodic();
    }
}
