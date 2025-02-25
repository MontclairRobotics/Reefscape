package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.GoToPoseCommand;
import frc.robot.util.TunerConstants;
import frc.robot.util.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.simulation.MapleSimSwerveDrivetrain;
import frc.robot.vision.Limelight;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
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
import frc.robot.util.Tunable;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.DynamicSlewRateLimiter;

public class Drivetrain extends TunerSwerveDrivetrain implements Subsystem {

    /*
     * 
     * CONSTANTS
     * owo :3
     */

    public static final double MAX_SPEED = 5; // TODO: actually set this with units
    public static double MAX_ROT_SPEED = 6.5;
    public static final double MIN_ROT_SPEED = Math.PI * (1.0 / 3.0);
    public static double FORWARD_ACCEL = 9; // m / s^2
    public static double SIDE_ACCEL = 9; // m / s^2
    public static double ROT_ACCEL = 9; // radians / s^2
    public static double MIN_TRANSLATIONAL_ACCEL = 0.5;
    public static double MIN_ROT_ACCEL = 0.3;
    public static boolean IS_LIMITING_ACCEL = true;

    DoublePublisher driveCurrentPub;
    DoublePublisher driveVelocityPub;


     /* Acceleration limiters for our drivetrain */
     private DynamicSlewRateLimiter forwardLimiter = new DynamicSlewRateLimiter(FORWARD_ACCEL); // TODO: actually set this
     private DynamicSlewRateLimiter strafeLimiter = new DynamicSlewRateLimiter(SIDE_ACCEL); // TODO: actually set this
     private DynamicSlewRateLimiter rotationLimiter = new DynamicSlewRateLimiter(ROT_ACCEL); // TODO: actually set this

    public Tunable forwardAccelTunable = new Tunable("Forward Accel Limit", 9, (value) -> forwardLimiter.setLimit(value));
    public Tunable sideAccelTunable = new Tunable("Side Accel Limit", 9, (value) -> strafeLimiter.setLimit(value));
    public Tunable rotAccelTunable = new Tunable("Rotation Accel Limit", 9, (value) -> rotationLimiter.setLimit(value));
    public Tunable rotMaxSpeedTunable = new Tunable("Rotation Max Speed", Math.PI * 1, (value) -> {
        MAX_ROT_SPEED = value;
    });
    public Tunable isLimitAccel = new Tunable("Is limiting accel", 1, (value) -> {
        if(value == 1) IS_LIMITING_ACCEL = true;
        if(value == 0) IS_LIMITING_ACCEL = false;
    });

    public static final Pose2d[] BLUE_SCORING_POSES = {
            // new Pose2d(new Translation2d(1.091, 1.060), new
            // Rotation2d(Math.toRadians(-127.000))), //top coral station
            // new Pose2d(new Translation2d(1.091, 7.000), new
            // Rotation2d(Math.toRadians(127.000))), //bottom coral station
            new Pose2d(new Translation2d(3.680, 2.600), new Rotation2d(Math.toRadians(60.000))),
            new Pose2d(new Translation2d(2.870, 4.030), new Rotation2d(Math.toRadians(0.000))),
            new Pose2d(new Translation2d(3.670, 5.450), new Rotation2d(Math.toRadians(-60.000))),
            new Pose2d(new Translation2d(5.320, 5.450), new Rotation2d(Math.toRadians(-120.000))),
            new Pose2d(new Translation2d(6.130, 4.030), new Rotation2d(Math.toRadians(180.000))),
            new Pose2d(new Translation2d(5.320, 2.600), new Rotation2d(Math.toRadians(120.000)))
    };

    public static final Pose2d[] LEFT_BLUE_SCORING_POSES = {
            new Pose2d(new Translation2d(3.17, 4.19), new Rotation2d(Math.toRadians(0))),
            new Pose2d(new Translation2d(3.98, 5.25), new Rotation2d(Math.toRadians(-60))),
            new Pose2d(new Translation2d(5.3, 5.09), new Rotation2d(Math.toRadians(-120))),
            new Pose2d(new Translation2d(5.8, 3.86), new Rotation2d(Math.toRadians(180))),
            new Pose2d(new Translation2d(5, 2.8), new Rotation2d(Math.toRadians(120))),
            new Pose2d(new Translation2d(3.69, 2.97), new Rotation2d(Math.toRadians(60))),
    };

    public static final Pose2d[] RIGHT_BLUE_SCORING_POSES = {
            new Pose2d(new Translation2d(3.17, 3.86), new Rotation2d(Math.toRadians(0))),
            new Pose2d(new Translation2d(3.69, 5.09), new Rotation2d(Math.toRadians(-60))),
            new Pose2d(new Translation2d(5, 5.25), new Rotation2d(Math.toRadians(-120))),
            new Pose2d(new Translation2d(5.8, 4.19), new Rotation2d(Math.toRadians(180))),
            new Pose2d(new Translation2d(5.3, 2.97), new Rotation2d(Math.toRadians(120))),
            new Pose2d(new Translation2d(3.98, 2.8), new Rotation2d(Math.toRadians(60))),
    };

    // for go to pose command
    private boolean isAtPoseGoal;
    private ProfiledPIDController xController;
    private ProfiledPIDController yController;
    private ProfiledPIDController PoseThetaController;
    private SwerveSetpointGenerator setpointGen;
    private SwerveSetpoint prevSetpoint;

    private Pose2d targetPose;

    /* Heading PID Controller for things like automatic alignment buttons */
    public PIDController thetaController = new PIDController(4.5, 0, .2);

    /* variable to store our heading */
    private Rotation2d odometryHeading;

    // private Pigeon2 gyro = this.getPigeon2(); //they say not to use this like
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

        resetPose(new Pose2d(3, 3, Rotation2d.fromDegrees(0)));

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
        prevSetpoint = new SwerveSetpoint(getCurrentSpeeds(), getState().ModuleStates, DriveFeedforwards.zeros(config.numModules));
    }

    /*
     * RETURNS X VELOCITY FROM CONTROLLER
     * lemon
     */
    public double getVelocityXFromController() {
        double xInput = -MathUtil.applyDeadband(RobotContainer.driverController.getLeftX(), 0.07);
        if (IS_LIMITING_ACCEL)
            return strafeLimiter.calculate(
                    Math.pow(xInput, 3) * MAX_SPEED);
        else
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

    /*
     * RETURNS Y VELOCITY FROM CONTROLLER
     * 
     */
    public double getVelocityYFromController() {
        double yInput = -MathUtil.applyDeadband(RobotContainer.driverController.getLeftY(), 0.07);
        if (IS_LIMITING_ACCEL)
            return forwardLimiter.calculate(Math.pow(yInput, 3) * MAX_SPEED);

        else
            return Math.pow(yInput, 3) * MAX_SPEED;

    }

    /*
     * DEFAULT DRIVE METHOD
     * 
     * 
     */
    public void driveJoystick() {
        double rotInput = -MathUtil.applyDeadband(RobotContainer.driverController.getRightX(), 0.07);
        double rotVelocity = Math.pow(rotInput, 3) * MAX_ROT_SPEED;
        //getMaxRotSpeed();
        if (IS_LIMITING_ACCEL) {
            rotVelocity = rotationLimiter.calculate(
                rotVelocity);
        }
        driveWithSetpoint(getVelocityYFromController(), getVelocityXFromController(), rotVelocity, fieldRelative, true); // drives
                                                                                                             // using
                                                                                                             // supposed
                                                                                                             // velocities,
                                                                                                             // rot
                                                                                                             // velocity,
                                                                                                             // and
                                                                                                             // field
                                                                                                             // relative
                                                                                                             // boolean
        // drive(0, 1, 0, true);
    }

    public void drive(ChassisSpeeds speeds, boolean fieldRelative, boolean respectOperatorPerspective) {
        drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, fieldRelative,
                respectOperatorPerspective);
    }

    /*
     * DRIVES GIVEN ROBOT RELATIVE CHASSIS SPEEDS, FOR PATHPLANNER
     * 
     * 
     */
    // public void drive(ChassisSpeeds chassisSpeeds){
    // //robot relative chassis speeds
    // SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds()
    // .withDriveRequestType(DriveRequestType.Velocity)
    // .withSteerRequestType(SteerRequestType.Position)
    // .withSpeeds(chassisSpeeds);

    // this.setControl(request);
    // }

    /*
     * DRIVES USING CLOSED LOOP VELOCITY CONTROL
     * 
     * 
     * 
     */
    public void drive(double velocityX, double velocityY, double rotationalVelocity, boolean fieldRelative,
            boolean respectOperatorPerspective) {
        if (fieldRelative) {
            SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
                    // .withDeadband(0.03) //TODO: set these
                    // .withRotationalDeadband(1)
                    .withDriveRequestType(DriveRequestType.Velocity) // Velocity is closed-loop velocity control
                    .withSteerRequestType(SteerRequestType.Position)
                    .withDesaturateWheelSpeeds(true); // TODO check

            if (!respectOperatorPerspective) {
                driveRequest = driveRequest.withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
            }

            this.setControl(
                    driveRequest
                            .withVelocityX(velocityX)
                            .withVelocityY(velocityY)
                            .withRotationalRate(rotationalVelocity));
        } else {
            // Creates robot relative swerve request
            SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
                    // .withDeadband(2) //TODO: set these
                    // .withRotationalDeadband(3)
                    .withDriveRequestType(DriveRequestType.Velocity) // Velocity is closed-loop velocity control
                    .withSteerRequestType(SteerRequestType.Position)
                    .withDesaturateWheelSpeeds(true); // TODO check

            // sets the control for the drivetrain
            this.setControl(
                    driveRequest
                            .withVelocityX(velocityX)
                            .withVelocityY(velocityY)
                            .withRotationalRate(rotationalVelocity));

        }
    }

    //TODO Check
    public void driveWithSetpoint(double xSpeed, double ySpeed, double thetaSpeed, boolean fieldRelative,
    boolean respectOperatorPerspective) {

        if (respectOperatorPerspective) {
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red && fieldRelative)  {
     
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
            0.02 // The loop time of the robot code, in seconds
        );




        SwerveRequest req = new SwerveRequest.ApplyRobotSpeeds()
                                    .withSpeeds(speeds)
                                    .withWheelForceFeedforwardsX(prevSetpoint.feedforwards().robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(prevSetpoint.feedforwards().robotRelativeForcesYNewtons());

        setControl(req);
    
        
    }

    public void driveWithSetpoint(ChassisSpeeds speeds, boolean fieldRelative, boolean respectOperatorPerspective) {
        driveWithSetpoint(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, fieldRelative, respectOperatorPerspective);
    }

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(2), // Reduce dynamic step voltage to 4 V to prevent brownout
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

    public Pose2d getClosestScoringPose(Pose2d[] posArr) {
        Pose2d closestPose;
        Pose2d currentPose;
        // if(!Robot.isReal())
        // currentPose = this.mapleSimSwerveDrivetrain.getSimulatedPose();
        currentPose = this.getRobotPose();
        System.out.println(currentPose);
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
                System.out.println("YAYA YAYAY AYAYAYClosest pose: " + closestPose);
            }
        }
        System.out.println("Closest pose: " + closestPose);
        return closestPose;
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
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(
                            // PID constants for translation
                            new PIDConstants(10, 0, 0),
                            // PID constants for rotation
                            new PIDConstants(7, 0, 0)),
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
        return AutoBuilder.pathfindToPose(getPoseToPathFindTo(RobotContainer.bottomLimelight), DEFAULT_CONSTRAINTS);
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
        resetRotation(Rotation2d.fromDegrees(0));
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

    public Pose2d getRobotPose() {
        return this.getState().Pose;
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return this.getState().Speeds;
    }

    @Override
    public void periodic() {

       // System.out.println(forwardAccelTunable.getValue());
        // Not sure if this is correct at all
        odometryHeading = this.getState().Pose.getRotation();
        isRobotAtAngleSetPoint = thetaController.atSetpoint();
        fieldRelative = !RobotContainer.driverController.L2().getAsBoolean();
        // strafeLimiter.setLimit(getMaxHorizontalAccel());
        // forwardLimiter.setLimit(getMaxForwardAccel());
        // rotationLimiter.setLimit(getMaxRotAccel());
        


        double driveSpeedModule0 = getState().ModuleStates[0].speedMetersPerSecond;
        double driveCurrentModule0 = getModule(0).getDriveMotor().getStatorCurrent().getValueAsDouble();
        driveCurrentPub.set(driveCurrentModule0);
        driveVelocityPub.set(driveSpeedModule0);
        

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
