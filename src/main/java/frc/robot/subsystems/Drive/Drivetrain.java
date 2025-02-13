package frc.robot.subsystems.Drive;
import frc.robot.RobotContainer;
import frc.robot.util.TunerConstants;
import frc.robot.util.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.simulation.MapleSimSwerveDrivetrain;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import static frc.robot.subsystems.Drive.DriveState.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Drivetrain extends TunerSwerveDrivetrain implements Subsystem {

    /* --------------------------------------- CONSTANTS ----------------------------------------------------------------- */

    public static final double MAX_SPEED = 5; //TODO: actually set this with units
    public static final double MAX_ROT_SPEED = Math.PI * 2;
    public static final double FORWARD_ACCEL_LIMIT = 9; // m / s^2
    public static final double SIDE_ACCEL_LIMIT = 9; //m / s^2
    public static final double ROT_ACCEL_LIMIT = 9; // radians / s^2

    public static final PathConstraints DEFAULT_CONSTRAINTS = 
        new PathConstraints(MAX_SPEED, FORWARD_ACCEL_LIMIT, MAX_ROT_SPEED, ROT_ACCEL_LIMIT);

    /* Acceleration limiters for our drivetrain */
    private SlewRateLimiter forwardLimiter = new SlewRateLimiter(FORWARD_ACCEL_LIMIT); //TODO: actually set this
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(SIDE_ACCEL_LIMIT); //TODO: actually set this
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(ROT_ACCEL_LIMIT); //TODO: actually set this
    
    public PIDController thetaController = new PIDController(0,0 ,0 ); //heading PID controller
    public DriveState state; //keeps track of driving state and other functions
    
    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    public Drivetrain() {
        super(
            TunerConstants.DrivetrainConstants, 
            TunerConstants.odometryUpdateFrequency, 
            TunerConstants.odometryStandardDeviation, 
            TunerConstants.visionStandardDeviation, 
            MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(
                new SwerveModuleConstants<?, ?, ?>[]{ 
                    TunerConstants.FrontLeft, 
                    TunerConstants.FrontRight, 
                    TunerConstants.BackLeft, 
                    TunerConstants.BackRight 
                }
            )
        );

        state = new DriveState();
        configurePathPlanner();
        resetPose(new Pose2d(3, 3, Rotation2d.fromDegrees(0))); //This is why our robot starts in a weird spot in SIM
        thetaController.setTolerance(1 * Math.PI / 180); //degrees converted to radians

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public DriveState getDriveState() {
        return this.state;
    }


    /* -------------------------------------------- DRIVING DURING TELEOP ------------------------------------------------------------- */
    
    
    /**
     * @return the velocity input in the x direction, cubed and limited by a SlewRateLimiter
     */
    public double getVelocityXFromController(){
        double xInput = -MathUtil.applyDeadband(RobotContainer.driverController.getLeftX(), 0.04);
        return forwardLimiter.calculate(Math.pow(xInput, 3) * MAX_SPEED);
    }

    /**
     * @return the velocity input in the Y direction, cubed and limited by a SlewRateLimiter
     */
    public double getVelocityYFromController(){
        double yInput = -MathUtil.applyDeadband(RobotContainer.driverController.getLeftY(), 0.04);
        return strafeLimiter.calculate(Math.pow(yInput, 3) * MAX_SPEED);
    }

    /**
     * Default drive method taking in joystick input
     */
    public void drive() {
        double rotInput = -MathUtil.applyDeadband(RobotContainer.driverController.getRightX(), 0.04);
        double rotVelocity = rotationLimiter.calculate(
            Math.pow(rotInput,3) * MAX_ROT_SPEED
            );
        drive(getVelocityYFromController(), getVelocityXFromController(), rotVelocity, isFieldRelative); //drives using supposed velocities, rot velocity, and field relative boolean
    }

    /** 
     * Default drive command
     */
    public Command driveJoystickInputCommand(){
        return Commands.run(() -> drive(), this);
    }

    /**
     * Drives using input chassisSpeeds, either field relative or robot relative
     */
    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, fieldRelative);
    }

    /* DRIVES GIVEN ROBOT RELATIVE CHASSIS SPEEDS, FOR PATHPLANNER 
     * 
     * 
    */
    // public void drive(ChassisSpeeds chassisSpeeds){
    //     //robot relative chassis speeds
    //     SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds()
    //     .withDriveRequestType(DriveRequestType.Velocity)
    //     .withSteerRequestType(SteerRequestType.Position)
    //     .withSpeeds(chassisSpeeds);

    //     this.setControl(request);
    // }

    /**
     * Drives using closed-loop velocity controll
     * @param velocityX the velocity input in the X direction
     * @param velocityY the velocity input in the Y direction
     * @param rotationalVelocity the rotation velocity input
     * @param fieldRelative whether or not to drive field relative
     */
    public void drive(double velocityX, double velocityY, double rotationalVelocity, boolean fieldRelative){
        if(fieldRelative){
            SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric() 
                // .withDeadband(0.03) //TODO: set these
                // .withRotationalDeadband(1) 
                .withDriveRequestType(DriveRequestType.Velocity) //Velocity is closed-loop velocity control
                .withSteerRequestType(SteerRequestType.Position); 
        
            this.setControl(
                driveRequest
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(rotationalVelocity)
            );
        } else {
            //Creates robot relative swerve request
            SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
                //.withDeadband(2) //TODO: set these
                //.withRotationalDeadband(3) 
                .withDriveRequestType(DriveRequestType.Velocity) //Velocity is closed-loop velocity control
                .withSteerRequestType(SteerRequestType.Position); 

            //sets the control for the drivetrain
            this.setControl(
                driveRequest
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(rotationalVelocity)
            );

        }
    }


    /* ------------------------------------------SYSTEM IDENTIFICATION -------------------------------------------------------------------------- */



    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(2), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

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


      /* --------------------------------------------- PATH PLANNER & AUTO ----------------------------------------- */
   
   
      //TODO: input correct poses
    public Pose2d getPoseToPathFindTo(){
            switch(RobotContainer.limelight.getTagID()){
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
                () -> this.getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> this.getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> this.setControl(
                    new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    // public PathPlannerPath generatePathToAprilTagLocation(){
    //     Pose2d finalPose = getPoseToPathFindTo();
    //     if(finalPose != null) {
    //         Pose2d currentPose = RobotContainer.limelight.getCurrentOdometryPose2d();
    //         List<PathPoint> pathList = Arrays.asList(
    //             new PathPoint(
    //                 currentPose.getTranslation(), 
    //                 new RotationTarget(
    //                     odometryHeading, 
    //                     new Rotation2d(Math.toRadians(odometryHeading))
    //                 )
    //             ), 
    //             new PathPoint(
    //                 finalPose.getTranslation(), 
    //                 new RotationTarget(
    //                     odometryHeading, 
    //                     finalPose.getRotation()
    //                 )
    //             )
    //         );
    //         GoalEndState goalEndState = new GoalEndState(0, finalPose.getRotation());
    //         //TODO: set these path constraints
    //         PathConstraints pathConstraints = new PathConstraints(null, null, null, null);
    //         return PathPlannerPath.fromPathPoints(pathList, pathConstraints, goalEndState);
    //     }    
    //     else return new PathPlannerPath(null, null, null, null);
    // }

    public Command followPathToAprilTagLocation(){
        return AutoBuilder.pathfindToPose(getPoseToPathFindTo(), DEFAULT_CONSTRAINTS);
    }


    /* ------------------------------------ AUTOMATIC ROTATION ----------------------------------------- */


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
     * Automatically turns the robot to the given angle
     * @param angDeg the angle in degress to align to, relative to the front of the robot
     */
    public void setRobotRelativeAngle(Rotation2d angDeg){
        double wrappedSetPoint = wrapAngle(odometryHeading.plus(angDeg)).getRadians();
        thetaController.setSetpoint(wrappedSetPoint);
    } 

   
   /**
    * Aligns to the angle set in {@link #setRobotRelativeAngle(Rotation2d)} 
    * @param lockDrive whether or not to lock the translation of the drivetrain
    */
    public void alignToAngleRobotRelative(boolean lockDrive) {
        //calculates the input for the drive function (NO IDEA IF I SHOULD MULTIPLY THIS BY SOMETHING)
        //inputs field relative angle (set point is also converted to field relative)
        double response = thetaController.calculate(getWrappedHeading().getRadians()) * Math.PI / 180;

        if(lockDrive) drive(0,0,response, false);
        else drive(getVelocityYFromController(), getVelocityXFromController(), response, false);
    }

    /**
     * Sets the field relative angle setpoint for automatic turning
     * @param angle the field relative angle to align to 
     */
    public void setFieldRelativeAngle(Rotation2d angle){
        double wrappedAngle = wrapAngle(angle).getRadians();
        thetaController.setSetpoint(wrappedAngle);
    }

    /**
     * Aligns to the angle set in {@link #setFieldRelativeAngle(Rotation2d)}
     * @param lockDrive whether or not to lock the translation of the drivetrain
     */
    public void alignToAngleFieldRelative(boolean lockDrive){
        //Response must be in Radians per second for the .drive method.
        double response = thetaController.calculate(getWrappedHeading().getRadians());
        if(lockDrive) drive(0,0, response, true);
        //swapped because pos X means forward
        else drive(getVelocityYFromController(), getVelocityXFromController(), response, true);
    }

    /**
     * Will continuously align to a changing angle
     * @param angleSup supplies the robot relative angle to align to 
     * @param lockDrive whether or not to lock the translation of the drivetrain
     */
    public void alignToAngleRobotRelativeContinuous(Supplier<Rotation2d> angleSup, boolean lockDrive){
        setRobotRelativeAngle(angleSup.get());
        alignToAngleRobotRelative(lockDrive);
    }

    /**
     * @param angle angle to align to in rotations, relative to the front of the robot
     * @param lockDrive whether or not to lock the translation of the drivetrain
     * @return
     */
    public Command alignToAngleRobotRelativeCommand(Rotation2d angle, boolean lockDrive) {
        return Commands.sequence(
            Commands.runOnce(() -> setRobotRelativeAngle(angle), RobotContainer.drivetrain),
            Commands.run(() -> alignToAngleRobotRelative(lockDrive), RobotContainer.drivetrain)
                .until(() -> isRobotAtAngleSetPoint)
        );
    }

    /**
     * @param angSup angle supplier in rotations
     * @param lockDrive whether or not to lock the translation of the drivetrain
     * @return
     */
    public Command alignToAngleRobotRelativeContinuousCommand(Supplier<Rotation2d> angSup, boolean lockDrive){
        return Commands.run(() -> alignToAngleRobotRelativeContinuous(angSup, lockDrive), this)
            .until(() -> isRobotAtAngleSetPoint);
    }
    
    /**
     * @param angle the field relative angle to align to, in rotations
     * @param lockDrive whether or not to lock the translation of the drivetrain
     * @return
     */
    public Command alignToAngleFieldRelativeCommand(Rotation2d angle, boolean lockDrive){
        return Commands.sequence(
            Commands.runOnce(() -> setFieldRelativeAngle(angle), RobotContainer.drivetrain),
            Commands.run(() -> alignToAngleFieldRelative(lockDrive), this)
                .until(() -> isRobotAtAngleSetPoint)
        );
    }

    @Override
    public void periodic(){
        //Not sure if this is correct at all
        // odometryHeading = this.getState().Pose.getRotation();
        // isRobotAtAngleSetPoint = thetaController.atSetpoint();
        // fieldRelative = !RobotContainer.driverController.L2().getAsBoolean();


        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        // if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
        //     DriverStation.getAlliance().ifPresent(allianceColor -> {
        //         setOperatorPerspectiveForward(
        //             allianceColor == Alliance.Red
        //                 ? kRedAlliancePerspectiveRotation
        //                 : kBlueAlliancePerspectiveRotation
        //         );
        //         m_hasAppliedOperatorPerspective = true;
        //     });
        // }
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
