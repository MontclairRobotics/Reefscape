package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.AlignToAprilTagCommandOffset;
import frc.robot.commands.DistanceAlign;
import frc.robot.commands.GoToPoseCommand;
import frc.robot.commands.GoToReefCommand;
import frc.robot.util.TunerConstants;
import frc.robot.util.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.simulation.MapleSimSwerveDrivetrain;
import frc.robot.vision.Limelight;
import frc.robot.vision.LimelightHelpers;
import static edu.wpi.first.math.util.Units.*;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.text.FieldPosition;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import frc.robot.util.PoseUtils;
import frc.robot.util.RobotState;
import frc.robot.util.TagOffset;
import frc.robot.util.Tunable;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.DynamicSlewRateLimiter;
import frc.robot.util.FieldPositionUtils;

public class Drivetrain extends TunerSwerveDrivetrain implements Subsystem {

    /*
     * 
     * CONSTANTS
     * owo :3
     */

    public static final double MAX_SPEED = 5; // TODO: actually set this with units
    public static double MAX_ROT_SPEED = 10;
    public static double MIN_ROT_SPEED = Math.PI * (1.0 / 3.0);
    public static double FORWARD_ACCEL = 9; // m / s^2
    public static double SIDE_ACCEL = 12; // m / s^2
    public static double ROT_ACCEL = 20; // radians / s^2
    public static double MIN_TRANSLATIONAL_ACCEL = 2;
    public static double MIN_ROT_ACCEL = 1.5;
    public static boolean IS_LIMITING_ACCEL = true; // TODO remove this, not needed w/ driveWithSetpoint

    public TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(3);

    DoublePublisher driveCurrentPub;
    DoublePublisher driveVelocityPub;

    /* Acceleration limiters for our drivetrain */
    private DynamicSlewRateLimiter forwardLimiter = new DynamicSlewRateLimiter(FORWARD_ACCEL); // TODO: actually set
                                                                                               // this
    private DynamicSlewRateLimiter strafeLimiter = new DynamicSlewRateLimiter(SIDE_ACCEL); // TODO: actually set this
    private DynamicSlewRateLimiter rotationLimiter = new DynamicSlewRateLimiter(ROT_ACCEL); // TODO: actually set this
    private Rotation2d targetHeading = Rotation2d.fromDegrees(0);

    public Tunable forwardAccelTunable = new Tunable("Forward Accel Limit", 1.5, (value) -> {
        // forwardLimiter.setLimit(value);
        // FORWARD_ACCEL = value;
        MIN_TRANSLATIONAL_ACCEL = value;
    });
    // public Tunable sideAccelTunable = new Tunable("Side Accel Limit", 1.5,
    // (value) -> {
    // //strafeLimiter.setLimit(value);
    // SIDE_ACCEL = value;
    // });
    public Tunable rotAccelTunable = new Tunable("Rotation Accel Limit", 0.3, (value) -> {
        // rotationLimiter.setLimit(value);
        // ROT_ACCEL = value;
        MIN_ROT_ACCEL = value;
    });

    public Tunable rotMaxSpeedTunable = new Tunable("Rotation MIN Speed", 1, (value) -> {
        MIN_ROT_SPEED = value;
    });
    // public Tunable isLimitAccel = new Tunable("Is limiting accel", 1, (value) ->
    // {
    // if(value == 1) IS_LIMITING_ACCEL = true;
    // if(value == 0) IS_LIMITING_ACCEL = false;
    // });

    AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public static Pose2d[] BLUE_SCORING_POSES;

    public static Pose2d[] LEFT_BLUE_SCORING_POSES;

    public static Pose2d[] RIGHT_BLUE_SCORING_POSES;

    public static final Pose2d[] BLUE_INTAKE_POSES = { // same as point 6 in pathplanner
        new Pose2d(new Translation2d(1.440, 0.772), new Rotation2d(Math.toRadians(-127.000))), // top coral station
        new Pose2d(new Translation2d(1.440, 7.2799016), new Rotation2d(Math.toRadians(127.000))), // bottom coral
    };

    // public static final Pose2d[] LEFT_BLUE_INTAKE_POSES = {
    //     new Pose2d(new Translation2d(1.66, .67), new Rotation2d(Math.toRadians(-127.000))), // top coral station
    //     new Pose2d(new Translation2d(.67, 6.65), new Rotation2d(Math.toRadians(127.000))), // bottom coral
    // };

    // public static final Pose2d[] RIGHT_BLUE_INTAKE_POSES = {
    //     new Pose2d(new Translation2d(.67, 1.39), new Rotation2d(Math.toRadians(-127.000))), // top coral station
    //     new Pose2d(new Translation2d(1.66, 7.36), new Rotation2d(Math.toRadians(127.000))), // bottom coral
    // };

    //used for driving with swerve setpoints, not used riht now
    private SwerveSetpointGenerator setpointGen;
    private SwerveSetpoint prevSetpoint;

    /* Heading PID Controller for things like automatic alignment buttons */
    public PIDController thetaController = new PIDController(5, 0, .1);

    /* variable to store our heading */
    public Rotation2d odometryHeading = new Rotation2d();

    // private Pigeon2 gyro = thifs.getPigeon2(); //they say not to use this like
    // this, allegedly
    // putting this here so we know how to get it
    private boolean isRobotAtAngleSetPoint; // for angle turning
    private boolean fieldRelative;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    // TODO: Set these values
    public static final PathConstraints DEFAULT_CONSTRAINTS = new PathConstraints(MAX_SPEED, FORWARD_ACCEL,
            MAX_ROT_SPEED, ROT_ACCEL);

    public Drivetrain() {

        super(
                TunerConstants.DrivetrainConstants,
                TunerConstants.odometryUpdateFrequency,
                TunerConstants.odometryStandardDeviation,
                TunerConstants.visionStandardDeviation,
                MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(
                        new SwerveModuleConstants<?, ?, ?>[] {
                                TunerConstants.FrontLeft,
                                TunerConstants.FrontRight,
                                TunerConstants.BackLeft,
                                TunerConstants.BackRight
                        }));
        if (Utils.isSimulation()) {
            startSimThread();
        }

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("Drive");
        driveCurrentPub = table.getDoubleTopic("Drive stator current").publish();
        driveVelocityPub = table.getDoubleTopic("Drive velocity").publish();

        thetaController.setTolerance(1 * Math.PI / 180); // degrees converted to radians
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        configurePathPlanner();

        resetPose(PoseUtils.flipPoseAlliance(new Pose2d(3, 3, Rotation2d.fromDegrees(0))));

        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        setpointGen = new SwerveSetpointGenerator(
                config, // The robot configuration. This is the same config used for generating
                        // trajectories and running path following commands.
                10.0 * 2 * Math.PI // The max rotation velocity of a swerve module in radians per second.
                                   // This should probably be stored in your Constants file
        );
        prevSetpoint = new SwerveSetpoint(getCurrentSpeeds(), getState().ModuleStates,
                DriveFeedforwards.zeros(config.numModules));

            AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
            field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
            BLUE_SCORING_POSES = new Pose2d[6];
            LEFT_BLUE_SCORING_POSES = new Pose2d[6];
            RIGHT_BLUE_SCORING_POSES = new Pose2d[6];
            double parallelDistance = 0.165;
            double perpendicularDistance = 0.48; // TODO set
            for (int i = 17; i <= 22; i++) {
                Pose3d tagPose = field.getTagPose(i).orElseGet(() -> new Pose3d()); // make better?
                Pose2d tagPose2d = new Pose2d(tagPose.getX(), tagPose.getY(), (Rotation2d.fromRadians(tagPose.getRotation().getZ()).minus(Rotation2d.k180deg)));
                
                Pose2d backPose = new Pose2d(tagPose.getX() - perpendicularDistance * tagPose2d.getRotation().getCos(), tagPose.getY() - perpendicularDistance * tagPose2d.getRotation().getSin(), tagPose2d.getRotation());
                BLUE_SCORING_POSES[i-17] = backPose;

                Pose2d rightPose = new Pose2d(backPose.getX() + parallelDistance * Math.cos((Math.PI / 2.0) - tagPose2d.getRotation().getRadians()), (backPose.getY() - parallelDistance * Math.sin((Math.PI / 2.0) - tagPose2d.getRotation().getRadians())), tagPose2d.getRotation());
                Pose2d leftPose = new Pose2d(backPose.getX() - (parallelDistance * Math.cos((Math.PI / 2.0) - tagPose2d.getRotation().getRadians())), backPose.getY() + parallelDistance * Math.sin((Math.PI / 2.0) - tagPose2d.getRotation().getRadians()), tagPose2d.getRotation());

                LEFT_BLUE_SCORING_POSES[i-17] = leftPose;
                RIGHT_BLUE_SCORING_POSES[i-17] = rightPose;

                System.out.println("id: " + i + " left: " + leftPose + " right: " + rightPose + " center: " + backPose);
                // double xTransform =  (parallelDistance * tagPose2d.getRotation().getCos());
                // double yTransform =  (perpendicularDistance * tagPose2d.getRotation().getSin());
                // double xTransform = parallelDistance * tagPose2d.getRotation().getCos() 
                //     - perpendicularDistance * tagPose2d.getRotation().getSin();
                // double yTransform = parallelDistance * tagPose2d.getRotation().getSin() 
                //     + perpendicularDistance * tagPose2d.getRotation().getCos();

                // System.out.println("xtransform: " + xTransform + " ytransform: " + yTransform);

                // Pose2d transformedPose = tagPose2d.transformBy(new Transform2d(xTransform, yTransform, Rotation2d.kZero));
                // System.out.println("tag pose: " + tagPose2d + " tag number: " + i + " Transformed pose: " + transformedPose);
                
            }
            // BLUE_SCORING_POSES = new Pose2d[]{
            //         // new Pose2d(new Translation2d(1.091, 1.060), new Rotation2d(Math.toRadians(-127.000))), // top coral station
            //         // new Pose2d(new Translation2d(1.091, 7.000), new Rotation2d(Math.toRadians(127.000))), // bottom coral
            //                                                                                               // station
            //         new Pose2d(new Translation2d(3.16, 4.04), new Rotation2d(Math.toRadians(0))),
            //         new Pose2d(new Translation2d(3.84, 5.15), new Rotation2d(Math.toRadians(-60.000))),
            //         new Pose2d(new Translation2d(5.15, 5.17), new Rotation2d(Math.toRadians(-120.000))),
            //         new Pose2d(new Translation2d(5.81, 4.04), new Rotation2d(Math.toRadians(180))),
            //         new Pose2d(new Translation2d(5.13, 2.88), new Rotation2d(Math.toRadians(120.000))),
            //         new Pose2d(new Translation2d(3.83, 2.90), new Rotation2d(Math.toRadians(60.000)))
            // };

        //     LEFT_BLUE_SCORING_POSES = new Pose2d[]{
        //         // new Pose2d(new Translation2d(1.66, .67), new Rotation2d(Math.toRadians(-127.000))), // top coral station
        //         // new Pose2d(new Translation2d(.67, 6.65), new Rotation2d(Math.toRadians(127.000))), // bottom coral
                 
        //         new Pose2d(new Translation2d(3.17, 4.19), new Rotation2d(Math.toRadians(0))),
        //         new Pose2d(new Translation2d(3.98, 5.24), new Rotation2d(Math.toRadians(-60))),
        //         new Pose2d(new Translation2d(5.27, 5.08), new Rotation2d(Math.toRadians(-120))),
        //         new Pose2d(new Translation2d(5.8, 3.87), new Rotation2d(Math.toRadians(180))),
        //         new Pose2d(new Translation2d(5.01, 2.775), new Rotation2d(Math.toRadians(120))),
        //         new Pose2d(new Translation2d(3.69, 2.97), new Rotation2d(Math.toRadians(60))),
        // };

    //     RIGHT_BLUE_SCORING_POSES = new Pose2d[]{
    //         // new Pose2d(new Translation2d(.67, 1.39), new Rotation2d(Math.toRadians(-127.000))), // top coral station
    //         // new Pose2d(new Translation2d(1.66, 7.36), new Rotation2d(Math.toRadians(127.000))), // bottom coral
             
    //         new Pose2d(new Translation2d(3.17, 3.87), new Rotation2d(Math.toRadians(0))),
    //         new Pose2d(new Translation2d(3.69, 5.09), new Rotation2d(Math.toRadians(-60))),
    //         new Pose2d(new Translation2d(5.01, 5.26), new Rotation2d(Math.toRadians(-120))),
    //         new Pose2d(new Translation2d(5.8, 4.19), new Rotation2d(Math.toRadians(180))),
    //         new Pose2d(new Translation2d(5.28, 2.97), new Rotation2d(Math.toRadians(120))),
    //         new Pose2d(new Translation2d(3.98, 2.8), new Rotation2d(Math.toRadians(60))),
    // };
    }

    /*
     * RETURNS X VELOCITY FROM CONTROLLER
     * lemon
     */
    public double getVelocityXFromController() {
        double xInput = -MathUtil.applyDeadband(RobotContainer.driverController.getLeftX(), 0.15);
        return Math.pow(xInput, 3) * MAX_SPEED;
    }

    public double getMaxForwardAccel() {
        double percentHeight = RobotContainer.elevator.getPercentHeight();
        return Math.max(-(FORWARD_ACCEL * percentHeight) + FORWARD_ACCEL, MIN_TRANSLATIONAL_ACCEL);
    }

    public double getMaxHorizontalAccel() {
        double percentHeight = RobotContainer.elevator.getPercentHeight();
        return Math.max(-(SIDE_ACCEL * percentHeight) + SIDE_ACCEL, MIN_TRANSLATIONAL_ACCEL);
    }

    public double getMaxRotAccel() {
        double percentHeight = RobotContainer.elevator.getPercentHeight();
        return Math.max(-(ROT_ACCEL * percentHeight) + ROT_ACCEL, MIN_ROT_ACCEL);
    }

    public double getMaxRotSpeed() {
        double percentHeight = RobotContainer.elevator.getPercentHeight();
        return Math.max(-(MAX_ROT_SPEED * percentHeight) + MAX_ROT_SPEED, MIN_ROT_SPEED);
    }

    double ROBOT_MASS = 68;
    double ROBOT_WIDTH = inchesToMeters(30);
    double COM_TO_CENTER_OF_ROTATION = inchesToMeters(Math.sqrt(1.5 * 1.5 + 0.55 * 0.55));

    // double COMX = (ROBOT_WIDTH / 2.0) + inchesToMeters(1.5); // back from front
    // of robot
    // double COMY = (ROBOT_WIDTH / 2.0) + inchesToMeters(0.55); // from side of
    // robot // COM approx centered -- good enough?
    public ChassisSpeeds getMaxSpeedsNoTip(ChassisSpeeds targetSpeeds) {
        double timestep = 0.02;

        double targetXSpeed = targetSpeeds.vxMetersPerSecond;
        double targetYSpeed = targetSpeeds.vyMetersPerSecond;
        double targetOmegaSpeed = targetSpeeds.omegaRadiansPerSecond;

        targetXSpeed = MathUtil.clamp(targetXSpeed, -MAX_SPEED, MAX_SPEED);
        targetYSpeed = MathUtil.clamp(targetYSpeed, -MAX_SPEED, MAX_SPEED);
        targetOmegaSpeed = MathUtil.clamp(targetOmegaSpeed, -MAX_ROT_SPEED, MAX_ROT_SPEED);

        ChassisSpeeds currentSpeeds = getCurrentSpeeds();

        double currentXSpeed = currentSpeeds.vxMetersPerSecond;
        double currentYSpeed = currentSpeeds.vyMetersPerSecond;
        double currentOmegaSpeed = currentSpeeds.omegaRadiansPerSecond;

        double targetXAccel = (targetXSpeed - currentXSpeed) / timestep;
        double targetYAccel = (targetYSpeed - currentYSpeed) / timestep;
        double targetRotAccel = (targetOmegaSpeed - currentOmegaSpeed) / timestep;

        // double maxAccel = (9.81 * (ROBOT_WIDTH / 2.0)) / getCOMHeight() * 0.9; //0.9
        // is safety factor // TODO use this or version below?
        double maxRotAccel = (9.81 * (ROBOT_WIDTH / 2.0)) / (getCOMHeight() * COM_TO_CENTER_OF_ROTATION) * 0.9;

        targetRotAccel = MathUtil.clamp(targetRotAccel, -maxRotAccel, maxRotAccel);

        // Always take max rotational accel over max drive (I think?)
        double maxAccel = Math
                .sqrt(Math.abs(Math.pow((9.81 * (ROBOT_WIDTH / 2.0)) / getCOMHeight(), 2)
                        - Math.pow(targetRotAccel * COM_TO_CENTER_OF_ROTATION, 2)))
                * 0.9;

        // System.out.println("Max Accel: " + maxAccel + "; Max Rot Accel:" +
        // maxRotAccel);
        // System.out.println(maxAccel);
        targetXAccel = MathUtil.clamp(targetXAccel, -maxAccel, maxAccel);
        targetYAccel = MathUtil.clamp(targetYAccel, -maxAccel, maxAccel);

        // TODO redundant, can get from accel? I think?
        double maxRotSpeed = Math.sqrt((9.81 * (ROBOT_WIDTH / 2)) / (COM_TO_CENTER_OF_ROTATION * getCOMHeight()));

        targetOmegaSpeed = MathUtil.clamp(targetOmegaSpeed, -maxRotSpeed, maxRotSpeed);

        targetXSpeed = (timestep * targetXAccel) + currentXSpeed;
        targetYSpeed = (timestep * targetYAccel) + currentYSpeed;

        // System.out.println("Target X Speed: " + targetSpeeds.vxMetersPerSecond +
        // "Limited Speed: " + targetXSpeed);
        Logger.recordOutput("Drive/TargetXSpeed", targetSpeeds.vxMetersPerSecond);
        Logger.recordOutput("Drive/LimitedXSpeed", targetXSpeed);

        Logger.recordOutput("Drive/TargetYSpeed", targetSpeeds.vyMetersPerSecond);
        Logger.recordOutput("Drive/LimitedYSpeed", targetYSpeed);

        Logger.recordOutput("Drive/TargetOmegaSpeed", targetSpeeds.omegaRadiansPerSecond);
        Logger.recordOutput("Drive/LimitedOmegaSpeed", targetOmegaSpeed);

        Logger.recordOutput("Drive/MaxAccel", maxAccel);
        Logger.recordOutput("Drive/MaxThetaAccel", maxRotAccel);

        Logger.recordOutput("Drive/TargetRotAccel", targetRotAccel);
        Logger.recordOutput("Drive/TargetXAccel", targetXAccel);
        Logger.recordOutput("Drive/TargetYAccel", targetYAccel);
        return new ChassisSpeeds(targetXSpeed, targetYSpeed, targetOmegaSpeed);
    }

    public Command driveToReefCommandFast(TagOffset direction) {
        BooleanSupplier isL4 = () -> {
            return RobotContainer.elevator.getTargetState() == RobotState.L4;
        };
        return Commands.sequence(
                Commands.print("IsL4: " + isL4.getAsBoolean()),
                Commands.parallel(
                        Commands.defer(() -> new GoToReefCommand(direction, !isL4.getAsBoolean()), Set.of(this)).andThen(Commands.print("Driving over")),
                        Commands.waitSeconds(0).andThen(Commands.defer(() -> RobotContainer.elevator.setState(RobotContainer.elevator.getTargetState()), Set.of(RobotContainer.elevator)).andThen(Commands.print("Elevator over")))),
                        Commands.print("Deferred BS over"),
                new GoToReefCommand(direction, false).unless(isL4));
    }

    public Command driveToReefCommandDistanceFast(TagOffset direction) {
        BooleanSupplier isL4 = () -> {
            return RobotContainer.elevator.getTargetState() == RobotState.L4;
        };
        Rotation2d targetRotation = getClosestTargetPose(BLUE_SCORING_POSES).getRotation();
        return Commands.sequence(
                Commands.print("IsL4: " + isL4.getAsBoolean()),
                alignToAngleFieldRelativeCommand(targetRotation, false),
                Commands.parallel(
                        Commands.defer(() -> new DistanceAlign(direction, !isL4.getAsBoolean()), Set.of(this)).andThen(Commands.print("Driving over")),
                        Commands.waitSeconds(0).andThen(Commands.defer(() -> RobotContainer.elevator.setState(RobotContainer.elevator.getTargetState()), Set.of(RobotContainer.elevator)).andThen(Commands.print("Elevator over")))),
                        Commands.print("Deferred BS over"),
                new DistanceAlign(direction, false).unless(isL4));
    }

    double ROBOT_COM_NO_ELEVATOR = inchesToMeters(5.8); // the COM of the robot and stage 1 of the elevator
    double ROBOT_MASS_NO_ELEVATOR = lbsToKilograms(115.34);

    double ROBOT_MASS_ELEVATOR = ROBOT_MASS - ROBOT_MASS_NO_ELEVATOR;

    public double getCOMHeight() {
        // return ROBOT_COM_NO_ELEVATOR +
        double elevatorCOMHeight = RobotContainer.elevator.getHeight() - inchesToMeters(11.1); // TODO make an
                                                                                               // interpolating tree map
                                                                                               // or something better
        // System.out.println(((ROBOT_MASS_NO_ELEVATOR * ROBOT_COM_NO_ELEVATOR) +
        // (ROBOT_MASS_ELEVATOR * elevatorCOMHeight)) / ROBOT_MASS);
        return ((ROBOT_MASS_NO_ELEVATOR * ROBOT_COM_NO_ELEVATOR) + (ROBOT_MASS_ELEVATOR * elevatorCOMHeight))
                / ROBOT_MASS;
    }

    /*
     * RETURNS Y VELOCITY FROM CONTROLLER
     * 
     */
    public double getVelocityYFromController() {
        double yInput = -MathUtil.applyDeadband(RobotContainer.driverController.getLeftY(), 0.15);
        return Math.pow(yInput, 3) * MAX_SPEED;
    }

    public boolean joystickInputDetected() {
        if (Math.abs(getVelocityXFromController()) > 0
                || Math.abs(getVelocityYFromController()) > 0) {
            return true;
        } else
            return false;
    }

    /*
     * DEFAULT DRIVE METHOD
     * 
     * 
     */
    public void driveJoystick() {
        double rotInput = -MathUtil.applyDeadband(RobotContainer.driverController.getRightX(), 0.15);
        double rotVelocity = Math.pow(rotInput, 3) * MAX_ROT_SPEED;

        drive(getVelocityYFromController(), getVelocityXFromController(), rotVelocity, fieldRelative, true); // drives
                                                                                                             // using
    }

    public void drive(ChassisSpeeds speeds, boolean fieldRelative, boolean respectOperatorPerspective) {
        drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, fieldRelative,
                respectOperatorPerspective);
    }

    /*
     * DRIVES USING CLOSED LOOP VELOCITY CONTROL
     * 
     * 
     * 
     */
    public void drive(double xSpeed, double ySpeed, double thetaSpeed, boolean fieldRelative,
            boolean respectOperatorPerspective) {

        if (IS_LIMITING_ACCEL) {
            thetaSpeed = rotationLimiter.calculate(thetaSpeed);
            xSpeed = forwardLimiter.calculate(xSpeed);
            ySpeed = strafeLimiter.calculate(ySpeed);
        }
        // System.out.println(new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed));

        // if (respectOperatorPerspective) {
        // if (DriverStation.getAlliance().isPresent() &&
        // DriverStation.getAlliance().get() == Alliance.Red && fieldRelative) {
        // velocityX *= -1;
        // velocityY *= -1;
        // }
        // }

        // if (fieldRelative) {
        // SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
        // // .withDeadband(0.03) //TODO: set these
        // // .withRotationalDeadband(1)
        // .withDriveRequestType(DriveRequestType.Velocity) // Velocity is closed-loop
        // velocity control
        // .withSteerRequestType(SteerRequestType.Position)
        // .withDesaturateWheelSpeeds(true); // TODO check

        // if (!respectOperatorPerspective) {
        // driveRequest =
        // driveRequest.withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
        // }

        // this.setControl(
        // driveRequest
        // .withVelocityX(velocityX)
        // .withVelocityY(velocityY)
        // .withRotationalRate(rotationalVelocity));
        // } else {
        // // Creates robot relative swerve request
        // SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
        // // .withDeadband(2) //TODO: set these
        // // .withRotationalDeadband(3)
        // .withDriveRequestType(DriveRequestType.Velocity) // Velocity is closed-loop
        // velocity control
        // .withSteerRequestType(SteerRequestType.Position)
        // .withDesaturateWheelSpeeds(true); // TODO check

        // // sets the control for the drivetrain
        // this.setControl(
        // driveRequest
        // .withVelocityX(velocityX)
        // .withVelocityY(velocityY)
        // .withRotationalRate(rotationalVelocity));

        // }

        if (respectOperatorPerspective) {
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
                    && fieldRelative) {

                xSpeed *= -1;
                ySpeed *= -1;
            }
        }
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);

        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getWrappedHeading());
        }

        // speeds = getMaxSpeedsNoTip(speeds);
        // System.out.println(getMaxSpeedsNoTip(speeds));

        SwerveRequest req = new SwerveRequest.ApplyRobotSpeeds()
                .withSpeeds(speeds)
                .withDriveRequestType(DriveRequestType.Velocity)
                .withSteerRequestType(SteerRequestType.Position);
                // .withWheelForceFeedforwardsX(prevSetpoint.feedforwards().robotRelativeForcesXNewtons())
                // .withWheelForceFeedforwardsY(prevSetpoint.feedforwards().robotRelativeForcesYNewtons());

        setControl(req);

    }

    public void driveWithSetpoint(double xSpeed, double ySpeed, double thetaSpeed, boolean fieldRelative,
            boolean respectOperatorPerspective) {
        driveWithSetpoint(xSpeed, ySpeed, thetaSpeed, fieldRelative, respectOperatorPerspective, true);
    }

    // TODO Check
    public void driveWithSetpoint(double xSpeed, double ySpeed, double thetaSpeed, boolean fieldRelative,
            boolean respectOperatorPerspective, boolean headingCorrection) {

        if (headingCorrection && Math.abs(thetaSpeed - 0.002) > 0) { // if angular speed commanded and heading
                                                                     // correction enabled
            targetHeading = odometryHeading; // update target heading
        }

        if (headingCorrection && Math.abs(thetaSpeed - 0.002) <= 0) {
            thetaSpeed = thetaController.calculate(odometryHeading.getRadians(), targetHeading.getRadians());
        }

        if (respectOperatorPerspective) {
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
                    && fieldRelative) {

                xSpeed *= -1;
                ySpeed *= -1;
            }
        }
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);

        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getWrappedHeading());
        }

        prevSetpoint = setpointGen.generateSetpoint(
                prevSetpoint, // The previous setpoint
                speeds, // The desired target speeds
                new PathConstraints(MAX_SPEED, getMaxForwardAccel(), getMaxRotSpeed(), getMaxRotAccel()),
                0.02 // The loop time of the robot code, in seconds
        );

        // speeds = getMaxSpeedsNoTip(speeds);
        // System.out.println(getMaxSpeedsNoTip(speeds));

        SwerveRequest req = new SwerveRequest.ApplyRobotSpeeds()
                .withSpeeds(prevSetpoint.robotRelativeSpeeds())
                .withDriveRequestType(DriveRequestType.Velocity)
                .withSteerRequestType(SteerRequestType.Position)
                .withWheelForceFeedforwardsX(prevSetpoint.feedforwards().robotRelativeForcesXNewtons())
                .withWheelForceFeedforwardsY(prevSetpoint.feedforwards().robotRelativeForcesYNewtons());

        setControl(req);

    }

    public void driveWithSetpoint(ChassisSpeeds speeds, boolean fieldRelative, boolean respectOperatorPerspective) {
        driveWithSetpoint(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond,
                fieldRelative, respectOperatorPerspective);
    }

    public void driveWithSetpoint(ChassisSpeeds speeds, boolean fieldRelative, boolean respectOperatorPerspective,
            boolean headingCorrection) {
        driveWithSetpoint(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond,
                fieldRelative, respectOperatorPerspective, headingCorrection);
    }

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    /**
     * WRAPS THE ANGLE FROM -180 TO 180 DEGREES
     */
    public static Rotation2d wrapAngle(Rotation2d ang) {
        double angle = ang.getDegrees();
        angle = (angle + 180) % 360; // Step 1 and 2
        if (angle < 0) {
            angle += 360; // Make sure it's positive
        }
        return Rotation2d.fromDegrees(angle - 180); // Step 3
    }

    public double getDistanceBetweenPoses(Pose2d pos1, Pose2d pos2) {
        return Math.sqrt(
                Math.pow(
                        (pos1.getX() - pos2.getX()), 2) +
                        Math.pow(
                                (pos1.getY() - pos2.getY()), 2));
    }

    public Pose2d getClosestTargetPose(Pose2d[] posArr) {
        Pose2d closestPose;
        Pose2d currentPose;
        currentPose = this.getRobotPose();
        // System.out.println(currentPose);
        closestPose = PoseUtils.flipPoseAlliance(posArr[0]);
        for (Pose2d pos : posArr) {
            pos = PoseUtils.flipPoseAlliance(pos);
            // System.out.println("Current pose: " + currentPose);
            // System.out.println("Distance between current and pos: " +
            // getDistanceBetweenPoses(currentPose, pos));
            // System.out.println("Distance between closest and current: " +
            // getDistanceBetweenPoses(closestPose, currentPose));
            double distanceFromCurrentToPose = currentPose.getTranslation().getDistance(pos.getTranslation());
            double distanceFromCurrentToClosest = currentPose.getTranslation()
                    .getDistance(closestPose.getTranslation());
            if (distanceFromCurrentToPose < distanceFromCurrentToClosest) {
                closestPose = pos;
                // System.out.println("YAYA YAYAY AYAYAYClosest pose: " + closestPose);
            }
        }
        // System.out.println("Closest pose: " + closestPose);
        return closestPose;
    }

    public int getClosestScoringPoseIndex() {
        Pose2d closestPose;
        Pose2d currentPose;
        int poseIndex = 0;
        currentPose = this.getRobotPose();
        closestPose = PoseUtils.flipPoseAlliance(BLUE_SCORING_POSES[0]);
        for (int i=0; i<BLUE_SCORING_POSES.length; i++) {
            Pose2d pos = PoseUtils.flipPoseAlliance(BLUE_SCORING_POSES[i]);
            double distanceFromCurrentToPose = currentPose.getTranslation().getDistance(pos.getTranslation());
            double distanceFromCurrentToClosest = currentPose.getTranslation().getDistance(closestPose.getTranslation());
            if (distanceFromCurrentToPose < distanceFromCurrentToClosest) {
                closestPose = pos;
                poseIndex = i;
            }
        }
        return poseIndex;
    }

    // TODO: input correct poses
    public Pose2d getPoseToPathFindTo(Limelight camera) {
        switch (camera.getTagID()) {
            case 12:
                return new Pose2d(new Translation2d(1.091, 1.060), new Rotation2d(Math.toRadians(-127.000)));
            case 13:
                return new Pose2d(new Translation2d(1.091, 7.000), new Rotation2d(Math.toRadians(127.000)));
            case 17:
                return new Pose2d(new Translation2d(3.680, 2.600), new Rotation2d(Math.toRadians(60.000)));
            case 18:
                return new Pose2d(new Translation2d(2.870, 4.030), new Rotation2d(Math.toRadians(0.000)));
            case 19:
                return new Pose2d(new Translation2d(3.670, 5.450), new Rotation2d(Math.toRadians(-60.000)));
            case 20:
                return new Pose2d(new Translation2d(5.320, 5.450), new Rotation2d(Math.toRadians(-120.000)));
            case 21:
                return new Pose2d(new Translation2d(6.130, 4.030), new Rotation2d(Math.toRadians(180.000)));
            case 22:
                return new Pose2d(new Translation2d(5.320, 2.600), new Rotation2d(Math.toRadians(120.000)));
            case 1:
                return new Pose2d(new Translation2d(), new Rotation2d());
            case 2:
                return new Pose2d(new Translation2d(), new Rotation2d());
            case 6:
                return new Pose2d(new Translation2d(), new Rotation2d());
            case 7:
                return new Pose2d(new Translation2d(), new Rotation2d());
            case 8:
                return new Pose2d(new Translation2d(), new Rotation2d());
            case 9:
                return new Pose2d(new Translation2d(), new Rotation2d());
            case 10:
                return new Pose2d(new Translation2d(), new Rotation2d());
            case 11:
                return new Pose2d(new Translation2d(), new Rotation2d());
        }
        return null;
    }

    private void configurePathPlanner() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> this.getState().Pose, // Supplier of current robot pose
                    this::resetPose, // Consumer for seeding pose against auto
                    () -> this.getState().Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> this.setControl(
                            new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds)
                                    .withSteerRequestType(SteerRequestType.Position)
                                    .withDriveRequestType(DriveRequestType.Velocity)
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(
                            // PID constants for translation
                            new PIDConstants(5, 0, 0),
                            // PID constants for rotation
                            new PIDConstants(5, 0, 0)),
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the
                    // case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }
    }

    // public PathPlannerPath generatePathToAprilTagLocation(){
    // Pose2d finalPose = getPoseToPathFindTo();
    // if(finalPose != null) {
    // Pose2d currentPose = RobotContainer.limelight.getCurrentOdometryPose2d();
    // List<PathPoint> pathList = Arrays.asList(
    // new PathPoint(
    // currentPose.getTranslation(),
    // new RotationTarget(
    // odometryHeading,
    // new Rotation2d(Math.toRadians(odometryHeading))
    // )
    // ),
    // new PathPoint(
    // finalPose.getTranslation(),
    // new RotationTarget(
    // odometryHeading,
    // finalPose.getRotation()
    // )
    // )
    // );
    // GoalEndState goalEndState = new GoalEndState(0, finalPose.getRotation());
    // //TODO: set these path constraints
    // PathConstraints pathConstraints = new PathConstraints(null, null, null,
    // null);
    // return PathPlannerPath.fromPathPoints(pathList, pathConstraints,
    // goalEndState);
    // }
    // else return new PathPlannerPath(null, null, null, null);
    // }

    public Command followPathToAprilTagLocation() {
        return AutoBuilder.pathfindToPose(getPoseToPathFindTo(RobotContainer.leftLimelight), DEFAULT_CONSTRAINTS);
    }

    /*
     * ROBOT RELATIVE SETPOINT METHOD
     * 
     */
    public void setRobotRelativeAngle(Rotation2d angDeg) {
        double wrappedSetPoint = wrapAngle(odometryHeading.plus(angDeg)).getRadians();
        thetaController.setSetpoint(wrappedSetPoint);
    }

    /*
     * ALIGN TO ANGLE ROBOT RELATIVE
     * 
     */
    public void alignToAngleRobotRelative(boolean lockDrive) {
        // calculates the input for the drive function (NO IDEA IF I SHOULD MULTIPLY
        // THIS BY SOMETHING)
        // inputs field relative angle (set point is also converted to field relative)
        double response = thetaController.calculate(odometryHeading.getRadians());

        if (lockDrive)
            drive(0, 0, response, false, true); // perspective doesn't matter in robot relative
        else
            drive(getVelocityYFromController(), getVelocityXFromController(), response, false, true);
    }

    /*
     * SETS ANGLE SETPOINT FOR FIELD RELATIVE TURNING
     * 
     */
    public void setFieldRelativeAngle(Rotation2d angle) {
        double wrappedAngle = wrapAngle(angle).getRadians();
        thetaController.setSetpoint(wrappedAngle);
    }

    /*
     * ALIGNS TO ANGLE FIELD RELATIVE
     * 
     */
    public void alignToAngleFieldRelative(boolean lockDrive) {
        // Response must be in Radians per second for the .drive method.
        double response = thetaController.calculate(odometryHeading.getRadians());
        if (lockDrive)
            drive(0, 0, response, true, true);
        // swapped because pos X means forward
        else
            drive(getVelocityYFromController(), getVelocityXFromController(), response, true, true);
    }

    /*
     * ALIGNS TO ANGLE ROBOT RELATIVE WITH CHANGING SETPOINT
     * 
     */
    public void alignToAngleRobotRelativeContinuous(Supplier<Rotation2d> angleSup, boolean lockDrive) {
        setRobotRelativeAngle(angleSup.get());
        alignToAngleRobotRelative(lockDrive);
    }

    public void toRobotRelative() {
        fieldRelative = false;
    }

    public void toFieldRelative() {
        fieldRelative = true;
    }

    public void zeroGyro() {
        // this.getPigeon2().setYaw(0);
        System.out.println("Zeroing Gyro");
        resetRotation(PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(0)));
    }

    /*
     * default drive command
     */
    public Command driveJoystickInputCommand() {
        return Commands.run(() -> driveJoystick(), this);
    }

    /* zeros the gyro */
    public Command zeroGyroCommand() {
        return Commands.runOnce(() -> zeroGyro(), RobotContainer.drivetrain);
    }

    /* turns boolean to robot relative */
    public Command toRobotRelativeCommand() {
        return Commands.runOnce(() -> toRobotRelative(), RobotContainer.drivetrain);
    }

    /* turns boolean to field relative */
    public Command toFieldRelativeCommand() {
        return Commands.runOnce(() -> toFieldRelative(), RobotContainer.drivetrain);
    }

    /* aligns to angle robot relative */
    public Command alignToAngleRobotRelativeCommand(Rotation2d angle, boolean lockDrive) {
        return Commands.sequence(
                Commands.runOnce(() -> setRobotRelativeAngle(angle), RobotContainer.drivetrain),
                Commands.run(() -> alignToAngleRobotRelative(lockDrive), RobotContainer.drivetrain)
                        .until(() -> isRobotAtAngleSetPoint));
    }

    /*
     * aligns to angle robot relative but angle can change
     * 
     */
    public Command alignToAngleRobotRelativeContinuousCommand(Supplier<Rotation2d> angle, boolean lockDrive) {
        return Commands.run(() -> alignToAngleRobotRelativeContinuous(angle, lockDrive), this)
                .until(() -> isRobotAtAngleSetPoint);
    }

    /* aligns to angle field relative! */
    public Command alignToAngleFieldRelativeCommand(Rotation2d angle, boolean lockDrive) {
        return Commands.sequence(
                Commands.runOnce(() -> setFieldRelativeAngle(angle), RobotContainer.drivetrain),
                Commands.run(() -> alignToAngleFieldRelative(lockDrive), this)
                        .until(() -> isRobotAtAngleSetPoint));
    }

    public Rotation2d getWrappedHeading() {
        return wrapAngle(odometryHeading);
    }

    @AutoLogOutput
    public Pose2d getRobotPose() {
        return this.getState().Pose;
    }

    @AutoLogOutput
    public ChassisSpeeds getCurrentSpeeds() {
        return this.getState().Speeds;
    }

    public Optional<Pose2d> getPoseAtTime(double time) {
        return poseBuffer.getSample(time);
    }

    @Override
    public void periodic() {
        // System.out.println(forwardAccelTunable.getValue());
        // Not sure if this is correct at all

        odometryHeading = getRobotPose().getRotation();
        isRobotAtAngleSetPoint = thetaController.atSetpoint();
        fieldRelative = !RobotContainer.driverController.L2().getAsBoolean();

        strafeLimiter.setLimit(getMaxHorizontalAccel());
        forwardLimiter.setLimit(getMaxForwardAccel());
        rotationLimiter.setLimit(getMaxRotAccel());

        if (DriverStation.isEnabled()) {
            poseBuffer.addSample(Timer.getFPGATimestamp(), getRobotPose());
        }

        if (DriverStation.isTeleopEnabled()) {
            Auto.field.setRobotPose(getRobotPose());
        }

        // Pose2d botPose = getRobotPose();
        // Logger.recordOutput("Drivetrain/PoseBeforeMoved", botPose);
        // Pose2d newPose = FieldPositionUtils.getNearestPositionOnField(botPose);
        // resetPose(newPose);
        // Logger.recordOutput("Drivetrain/PoseAfterReset", newPose);

        Logger.recordOutput("Drivetrain/isFieldRelative", fieldRelative);

        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private Notifier m_simNotifier = null;
    private MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;

    @SuppressWarnings("unchecked")
    private void startSimThread() {
        mapleSimSwerveDrivetrain = new MapleSimSwerveDrivetrain(
                Units.Seconds.of(0.002),
                // TODO: modify the following constants according to your robot
                Units.Pounds.of(60), // robot weight
                Units.Inches.of(35.5), // bumper length
                Units.Inches.of(35.5), // bumper width
                DCMotor.getKrakenX60Foc(1), // drive motor type
                DCMotor.getKrakenX60Foc(1), // steer motor type
                1.2, // wheel COF
                getModuleLocations(),
                getPigeon2(),
                getModules(),
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight);
        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
        m_simNotifier.startPeriodic(0.002);
    }
}
