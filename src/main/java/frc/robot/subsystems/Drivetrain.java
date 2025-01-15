package frc.robot.subsystems;
import frc.robot.RobotContainer;

import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.SwerveDriveBrake;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.DriveConfig;

public class Drivetrain extends SubsystemBase {

    /*
     * 
     * CONSTANTS
     * 
     */

    public static final double MAX_SPEED = 4; //TODO: actually set this with units
    public static final double MAX_ROT_SPEED = Math.PI*2;
    public static final double FORWARD_ACCEL = 3; // m / s^2
    public static final double SIDE_ACCEL = 3; //m / s^2
    public static final double ROT_ACCEL = 2; // radians / s^2
    
    /* Swerve Drive Object */
    public final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerveDrive = DriveConfig.createSwerveDrivetrain();    
   
    /* Acceleration limiters for our drivetrain */
    private SlewRateLimiter forwardLimiter = new SlewRateLimiter(FORWARD_ACCEL); //TODO: actually set this
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(SIDE_ACCEL); //TODO: actually set this
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(ROT_ACCEL); //TODO: actually set this
    
    /* Heading PID Controller for things like automatic alignment buttons */
    public PIDController thetaController = new PIDController(0,0 ,0 );
    
    /* variable to store our heading  */
    private Rotation2d odometryHeading;
    // private Pigeon2 gyro = swerveDrive.getPigeon2(); //they say not to use this like this, allegedly
    //putting this here so we know how to get it
    private boolean isRobotAtAngleSetPoint; //for angle turning
    private boolean fieldRelative;

    //TODO: Set these values
    public static final PathConstraints DEFAULT_CONSTRAINTS = new PathConstraints(null, null, null, null);


    public Drivetrain() {

        thetaController.setTolerance(1 * Math.PI / 180); //degrees converted to radians
        configurePathPlanner();

    }

    /* RETURNS X VELOCITY FROM CONTROLLER 
     * 
    */
    public double getVelocityXFromController(){
        double xInput = RobotContainer.driverController.getLeftX();
        return forwardLimiter.calculate(Math.pow(xInput, 3) * MAX_SPEED);
    }

    /* RETURNS Y VELOCITY FROM CONTROLLER 
     * 
    */
    public double getVelocityYFromController(){
        double yInput = RobotContainer.driverController.getLeftY();
        return strafeLimiter.calculate(Math.pow(yInput, 3) * MAX_SPEED);
    }

    /* DEFAULT DRIVE METHOD
     * 
     * 
     */
    public void drive() {
        double rotInput = RobotContainer.driverController.getRightX();
        double rotVelocity = rotationLimiter.calculate(Math.pow(rotInput,3) * MAX_ROT_SPEED);
        drive(getVelocityYFromController(), getVelocityXFromController(), rotVelocity, fieldRelative); //drives using supposed velocities, rot velocity, and field relative boolean
    }

    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, fieldRelative);
    }

    /* DRIVES GIVEN ROBOT RELATIVE CHASSIS SPEEDS, FOR PATHPLANNER 
     * 
     * 
    */
    public void drive(ChassisSpeeds chassisSpeeds){
        //robot relative chassis speeds
        SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.Position)
        .withSpeeds(chassisSpeeds);

        swerveDrive.setControl(request);
    }

    /* DRIVES USING CLOSED LOOP VELOCITY CONTROL 
     * 
     * 
     * 
    */
    public void drive(double velocityX, double velocityY, double rotationalVelocity, boolean fieldRelative){
        if(fieldRelative){
            SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric() 
                //.withDeadband(2) //TODO: set these
                //.withRotationalDeadband(3) 
                .withDriveRequestType(DriveRequestType.Velocity) //Velocity is closed-loop velocity control
                .withSteerRequestType(SteerRequestType.Position); 
        
            swerveDrive.setControl(
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
            swerveDrive.setControl(
                driveRequest
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(rotationalVelocity)
            );

        }
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
                () -> swerveDrive.getState().Pose,   // Supplier of current robot pose
                swerveDrive::resetPose,         // Consumer for seeding pose against auto
                () -> swerveDrive.getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> swerveDrive.setControl(
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
    /* ROBOT RELATIVE SETPOINT METHOD 
     * 
    */
  public void setRobotRelativeAngle(Rotation2d angDeg){
    double wrappedSetPoint = wrapAngle(odometryHeading.plus(angDeg)).getRadians();
    thetaController.setSetpoint(wrappedSetPoint);
  }

   /* ALIGN TO ANGLE ROBOT RELATIVE 
    * 
   */
  public void alignToAngleRobotRelative(boolean lockDrive) {
        //calculates the input for the drive function (NO IDEA IF I SHOULD MULTIPLY THIS BY SOMETHING)
        //inputs field relative angle (set point is also converted to field relative)
        double response = thetaController.calculate(odometryHeading.getRadians()) * Math.PI / 180;

        if(lockDrive) drive(0,0,response, false);
        else drive(getVelocityYFromController(), getVelocityXFromController(), response, false);
    }


    /* SETS ANGLE SETPOINT FOR FIELD RELATIVE TURNING
     * 
     */
    public void setFieldRelativeAngle(Rotation2d angle){
        double wrappedAngle = wrapAngle(angle).getRadians();
        thetaController.setSetpoint(wrappedAngle);
    }

    /* ALIGNS TO ANGLE FIELD RELATIVE 
     * 
     */
    public void alignToAngleFieldRelative(boolean lockDrive){
        //Response must be in Radians per second for the .drive method.
        double response = thetaController.calculate(odometryHeading.getRadians());
        if(lockDrive) drive(0,0, response, true);
        //swapped because pos X means forward
        else drive(getVelocityYFromController(), getVelocityXFromController(), response, true);
    }

    /* ALIGNS TO ANGLE ROBOT RELATIVE WITH CHANGING SETPOINT
     * 
     */
    public void alignToAngleRobotRelativeContinuous(Supplier<Rotation2d> angleSup, boolean lockDrive){
        setRobotRelativeAngle(angleSup.get());
        alignToAngleRobotRelative(lockDrive);
    }

    public void toRobotRelative(){
        fieldRelative = false;
    }
    public void toFieldRelative(){
        fieldRelative = true;
    }
    public void zeroGyro(){
        swerveDrive.getPigeon2().setYaw(0);
    }

    /*
     * default drive command
     */
    public Command driveJoystickInputCommand(){
        return Commands.run(() -> drive(), this);
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
    public Command toFieldRelativeCommand (){
        return Commands.runOnce(() -> toFieldRelative(), RobotContainer.drivetrain);
    }

    /* aligns to angle robot relative */
    public Command alignToAngleRobotRelativeCommand(Rotation2d angle, boolean lockDrive) {
        return Commands.sequence(
            Commands.runOnce(() -> setRobotRelativeAngle(angle), RobotContainer.drivetrain),
            Commands.run(() -> alignToAngleRobotRelative(lockDrive), RobotContainer.drivetrain)
                .until(() -> isRobotAtAngleSetPoint)
        );
    }

    /* aligns to angle robot relative but angle can change
     * 
     */
    public Command alignToAngleRobotRelativeContinuousCommand(Supplier<Rotation2d> angle, boolean lockDrive){
        return Commands.run(() -> alignToAngleRobotRelativeContinuous(angle, lockDrive), this)
            .until(() -> isRobotAtAngleSetPoint);
    }
    
    /* aligns to angle field relative! */
    public Command alignToAngleFieldRelative(Rotation2d angle, boolean lockDrive){
        return Commands.sequence(
            Commands.runOnce(() -> setFieldRelativeAngle(angle), RobotContainer.drivetrain),
            Commands.run(() -> alignToAngleFieldRelative(lockDrive), this)
                .until(() -> isRobotAtAngleSetPoint)
        );
    }

    public Rotation2d getWrappedHeading() {
        return wrapAngle(odometryHeading);
    }

    public Pose2d getRobotPose() {
        return swerveDrive.getState().Pose;
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return swerveDrive.getState().Speeds;
    }

    @Override
    public void periodic(){
        //Not sure if this is correct at all
        odometryHeading = swerveDrive.getState().Pose.getRotation();
        isRobotAtAngleSetPoint = thetaController.atSetpoint();
        fieldRelative = !RobotContainer.driverController.L2().getAsBoolean();
    }
}
