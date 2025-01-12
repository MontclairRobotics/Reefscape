package frc.robot.Subsystems;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Vision.Limelight;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

import java.time.chrono.IsoChronology;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConfig;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
    
    /* NOTE
     * 
     * ODOMETRY AND SETTING FIELD/ROBOT RELATIVE ANGLES ARE HANDLED IN DEGREES
     * DEGREES!!!!
     * 
     * HOWEVER, CTRE swerve library CALCULATES THINGS IN RADIANS PER SECOND
     * ALWAYS CONVERT TO RADIANS BEFORE USING SWERVE REQUEST
     * 
     */
    public final SwerveDrivetrain swerveDrivetrain = DriveConfig.createSwerveDrivetrain();    
    private SlewRateLimiter forwardLimiter = new SlewRateLimiter(3); //TODO: actually set this
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3); //TODO: actually set this
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3); //TODO: actually set this
    private PIDController thetaController = new PIDController(0,0 ,0 );
    private double odometryHeading;
    private Pigeon2 gyro = swerveDrivetrain.getPigeon2(); //they say not to use this like this, allegedly
    //putting this here so we know how to get it
    private boolean isRobotAtAngleSetPoint; //for angle turning
    private boolean fieldRelative;


    public Drivetrain() {
        thetaController.setTolerance(1); //degrees
        

    }

    /* RETURNS X VELOCITY FROM CONTROLLER 
     * 
    */
    public double getVelocityXFromController(){
        double xInput = RobotContainer.driverController.getLeftX();
        return forwardLimiter.calculate(Math.pow(xInput, 3) * DriveConstants.MAX_SPEED);
    }

    /* RETURNS Y VELOCITY FROM CONTROLLER 
     * 
    */
    public double getVelocityYFromController(){
        double yInput = RobotContainer.driverController.getLeftY();
        return strafeLimiter.calculate(Math.pow(yInput, 3) * DriveConstants.MAX_SPEED);
    }

    /* DEFAULT DRIVE METHOD
     * 
     * 
     */
    public void drive() {
        double rotInput = RobotContainer.driverController.getRightX();
        double rotVelocity = rotationLimiter.calculate(Math.pow(rotInput,3) * DriveConstants.MAX_ROT_SPEED);
        drive(getVelocityYFromController(), getVelocityXFromController(), rotVelocity, fieldRelative); //drives using supposed velocities, rot velocity, and field relative boolean
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

        swerveDrivetrain.setControl(request);
    }

    //rotation in radians per second
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
        
            swerveDrivetrain.setControl(
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
            swerveDrivetrain.setControl(
                driveRequest
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(rotationalVelocity)
            );

        }
    }

    /* WRAPS THE ANGLE FROM -180 TO 180 DEGREES 
     * 
    */
    public static double wrapAngle(double angle) {
        angle = (angle + 180) % 360; // Step 1 and 2
        if (angle < 0) {
            angle += 360; // Make sure it's positive
        }
        return angle - 180; // Step 3
      }

    public Pose2d getPose2dToPathFindTo(){
        if(RobotContainer.isBlueAlliance){
            switch(RobotContainer.limelight.tagID){
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
            }
            return null;
        } else {

            //TODO: add these poses
            switch(RobotContainer.limelight.tagID){
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
    }

    public PathPlannerPath generatePathToAprilTagLocation(){
        Pose2d finalPose = getPose2dToPathFindTo();
        if(finalPose != null) {
            Pose2d currentPose = RobotContainer.limelight.getCurrentOdometryPose2d();
            List<PathPoint> pathList = Arrays.asList(
                new PathPoint(
                    currentPose.getTranslation(), 
                    new RotationTarget(
                        odometryHeading, 
                        new Rotation2d(Math.toRadians(odometryHeading))
                    )
                ), 
                new PathPoint(
                    finalPose.getTranslation(), 
                    new RotationTarget(
                        odometryHeading, 
                        finalPose.getRotation()
                    )
                )
            );
            GoalEndState goalEndState = new GoalEndState(0, finalPose.getRotation());
            //TODO: set these path constraints
            PathConstraints pathConstraints = new PathConstraints(null, null, null, null);
            return PathPlannerPath.fromPathPoints(pathList, pathConstraints, goalEndState);
        }    
        else return new PathPlannerPath(null, null, null, null);
    }

    public void followPathToAprilTagLocation(){
        //TODO: follow the path generated by generatePathToAprilTagLocation
    }
    /* ROBOT RELATIVE SETPOINT METHOD 
     * 
    */
  public void setRobotRelativeAngle(double angDeg){
    double wrappedSetPoint = wrapAngle(odometryHeading + angDeg);
    thetaController.setSetpoint(wrappedSetPoint);
  }

   /* ALIGN TO ANGLE ROBOT RELATIVE 
    * 
   */
  public void alignToAngleRobotRelative(boolean lockDrive) {
    //creates a blank translation to pass in to the drive function so the robot doesn't move
    Translation2d translation = new Translation2d(0, 0);
    //calculates the input for the drive function (NO IDEA IF I SHOULD MULTIPLY THIS BY SOMETHING)
    //inputs field relative angle (set point is also converted to field relative)
    double response = thetaController.calculate(odometryHeading) * Math.PI / 180;

    if(lockDrive) drive(0,0,response, false);
    else drive(getVelocityYFromController(), getVelocityXFromController(), response, false);
    }


    /* SETS ANGLE SETPOINT FOR FIELD RELATIVE TURNING
     * 
     */
    public void setFieldRelativeAngle(double angDeg){
        double wrappedAngle = wrapAngle(angDeg);
        thetaController.setSetpoint(wrappedAngle);
    }

    /* ALIGNS TO ANGLE FIELD RELATIVE 
     * 
     */
    public void alignToAngleFieldRelative(boolean lockDrive){
        //Response must be in Radians per second for the .drive method.
        double response = thetaController.calculate(odometryHeading) * Math.PI/180;
        if(lockDrive) drive(0,0, response, true);
        //swapped because pos X means forward
        else drive(getVelocityYFromController(), getVelocityXFromController(), response, true);
    }

    /* ALIGNS TO ANGLE ROBOT RELATIVE WITH CHANGING SETPOINT
     * 
     */
    public void alignToAngleRobotRelativeContinuos(DoubleSupplier angDeg, boolean lockDrive){
        setRobotRelativeAngle(angDeg.getAsDouble());
        alignToAngleRobotRelative(lockDrive);
    }

    public void toRobotRelative(){
        fieldRelative = false;
    }
    public void toFieldRelative(){
        fieldRelative = true;
    }
    public void zeroGyro(){
        swerveDrivetrain.getPigeon2().setYaw(0);
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
    public Command alignToAngleRobotRelativeCommand(double angDeg, boolean lockDrive) {
        return Commands.sequence(
            Commands.runOnce(() -> setRobotRelativeAngle(angDeg), RobotContainer.drivetrain),
            Commands.run(() -> alignToAngleRobotRelative(lockDrive), RobotContainer.drivetrain)
                .until(() -> isRobotAtAngleSetPoint)
        );
    }

    /* aligns to angle robot relative but angle can change
     * 
     */
    public Command alignToAngleRobotRelativeContinuosCommand(DoubleSupplier angDeg, boolean lockDrive){
        return Commands.run(() -> alignToAngleRobotRelativeContinuos(angDeg, lockDrive), this)
            .until(() -> isRobotAtAngleSetPoint);
    }
    
    /* aligns to angle field relative! */
    public Command alignToAngleFieldRelative(double angle, boolean lockDrive){
        return Commands.sequence(
            Commands.runOnce(() -> setFieldRelativeAngle(angle), RobotContainer.drivetrain),
            Commands.run(() -> alignToAngleFieldRelative(lockDrive))
                .until(() -> isRobotAtAngleSetPoint)
        );
    }

    @Override
    public void periodic(){
        //Not sure if this is correct at all
        odometryHeading = swerveDrivetrain.getPigeon2().getYaw().getValueAsDouble();
        isRobotAtAngleSetPoint = thetaController.atSetpoint();
        fieldRelative = !RobotContainer.driverController.L2().getAsBoolean();

        RobotContainer.isBlueAlliance = DriverStation.getAlliance().get() == Alliance.Red? false: true;
    }
}
