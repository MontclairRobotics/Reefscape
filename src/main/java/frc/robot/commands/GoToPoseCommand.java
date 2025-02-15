// package frc.robot.commands;

// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.networktables.DoublePublisher;
// import edu.wpi.first.networktables.DoubleTopic;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.units.Units;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.util.TunerConstants;

// public class GoToPoseCommand extends Command {

//     private ProfiledPIDController xController;
//     private ProfiledPIDController yController;
//     private ProfiledPIDController thetaController;

//     private Pose2d targetPose;

//     DoublePublisher xOutputPub;
//     DoublePublisher yOutputPub;
//     DoublePublisher rotOutputPub;

//     DoublePublisher xPosePub;
//     DoublePublisher yPosePub;
//     DoublePublisher rotPosePub;

//     @Override
//     public void initialize() {

//         targetPose = RobotContainer.drivetrain.getClosestScoringPose();
//         NetworkTableInstance inst = NetworkTableInstance.getDefault();
//         NetworkTable poseCommandTable = inst.getTable("Pose Command");

//         DoubleTopic xSetpointTopic = poseCommandTable.getDoubleTopic("X Setpoint");
//         DoublePublisher xSetpointPub = xSetpointTopic.publish();
//         DoubleTopic ySetpointTopic = poseCommandTable.getDoubleTopic("Y Setpoint");
//         DoublePublisher ySetpointPub = ySetpointTopic.publish();
//         DoubleTopic rotSetpointTopic = poseCommandTable.getDoubleTopic("Rot Setpoint");
//         DoublePublisher rotSetpointPub = rotSetpointTopic.publish();

//         xSetpointPub.set(targetPose.getX());
//         ySetpointPub.set(targetPose.getY());
//         rotSetpointPub.set(targetPose.getRotation().getRadians());

//         DoubleTopic xOutputTopic = poseCommandTable.getDoubleTopic("X Output");
//         xOutputPub = xOutputTopic.publish();
//         DoubleTopic yOutputTopic = poseCommandTable.getDoubleTopic("Y Output");
//         yOutputPub = yOutputTopic.publish();
//         DoubleTopic rotOutputTopic = poseCommandTable.getDoubleTopic("Rot Output");
//         rotOutputPub = rotOutputTopic.publish();

//         DoubleTopic xPoseTopic = poseCommandTable.getDoubleTopic("X Pose");
//         xPosePub = xPoseTopic.publish();
//         DoubleTopic yPoseTopic = poseCommandTable.getDoubleTopic("Y Pose");
//         yPosePub = yPoseTopic.publish();
//         DoubleTopic rotPoseTopic = poseCommandTable.getDoubleTopic("Rot Pose");
//         rotPosePub = rotPoseTopic.publish();
        
//         xController.setGoal(targetPose.getX());
//         yController.setGoal(targetPose.getY());
//         thetaController.setGoal(targetPose.getRotation().getRadians());
//     }

//     public GoToPoseCommand() {
//         addRequirements(RobotContainer.drivetrain);
//         xController = new ProfiledPIDController(5, 0, 1, new TrapezoidProfile.Constraints(
//                 TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond), Drivetrain.FORWARD_ACCEL));
//         yController = new ProfiledPIDController(5, 0, 1, new TrapezoidProfile.Constraints(
//                 TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond), Drivetrain.SIDE_ACCEL));
//         thetaController = new ProfiledPIDController(RobotContainer.drivetrain.thetaController.getP(),
//                 RobotContainer.drivetrain.thetaController.getI(), RobotContainer.drivetrain.thetaController.getD(),
//                 new TrapezoidProfile.Constraints(Drivetrain.MAX_ROT_SPEED, Drivetrain.ROT_ACCEL));
//         thetaController.enableContinuousInput(-Math.PI, Math.PI);
//         //targetPose = RobotContainer.drivetrain.getClosestScoringPose();
//         System.out.println("Target Pose" + targetPose);
//     }

//     @Override
//     public void execute() {
//         ChassisSpeeds speeds = RobotContainer.drivetrain.getCurrentSpeeds();
//         Pose2d currentPose = RobotContainer.drivetrain.getState().Pose;

//         xPosePub.set(currentPose.getX());
//         yPosePub.set(currentPose.getY());
//         rotPosePub.set(currentPose.getRotation().getRadians());

//         double xSpeed = xController.calculate(currentPose.getX());
//         double ySpeed = yController.calculate(currentPose.getY());
//         double omegaSpeed = thetaController.calculate(currentPose.getRotation().getRadians());

//         RobotContainer.drivetrain.drive(xSpeed, ySpeed, omegaSpeed, true);

//     }

//     @Override
//     public void end(boolean interrupted) {
//         // Needed?
//         RobotContainer.drivetrain.drive(0, 0, 0, true);

//     }

//     @Override
//     public boolean isFinished() {
//         // .atSetpoint?
//         return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
//     }

// }
