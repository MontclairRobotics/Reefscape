package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.RobotState;
import frc.robot.util.Tunable;
import frc.robot.util.Elastic;
import frc.robot.util.PoseUtils;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import frc.robot.util.simulation.DoubleJointedArmModel;

public class Arm extends SubsystemBase {

    public double armLimitVoltage = 1.7;
    public final double MAX_VELOCITY = 60.0 / 360.0; // rotations per sec
    public final double MAX_ACCELERATION = 20.0 / 360.0; // rotations per sec per sec

    // The max safe angle of the endpoint to the horizontal 
    public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(0); //TODO: FIND
                                                                              
    // The min safe angle of the endpoint to the horizontal
    public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0); 

    // TODO: grab value from real robot using protractor

    private ArmFeedforward armFeedforward = new ArmFeedforward(0, 0.2, 0); //TODO: find

    private PIDController pidController = new PIDController(80, 10, 3);//TODO: find

    public Tunable armVoltageLimit = new Tunable("Arm Limit Voltage", 1, (val)->{
        armLimitVoltage = val;
    });
    
    private SlewRateLimiter accelLimiter = new SlewRateLimiter(8); // accelerate fully in ~1.5 seconds (can tune value)

    private final double SLOW_DOWN_ZONE = 7.0; // The percent at the extreme angles where the arm will slow down to avoid
                                               // crashing into the elevator during manual control
    private final double SLOWEST_SPEED = 0.3;  // TODO: probs can take this out because we dont need to allow the arm to move 24/7

    private DutyCycleEncoder encoder;

    DutyCycleEncoderSim encoderSim;
    private SparkMax armMotor;
    private SparkMaxSim armMotorSim;
    // public double smallWristAngle;
    // public double largeWristAngle;

    private DoublePublisher voltagePub;
    private DoublePublisher RotPub;

    private DoublePublisher setpointPub;
    private DoublePublisher percentRotPub;

    private StructPublisher<Pose3d> elbowPosePub;
    private StructPublisher<Pose3d> posePub;

    public Tunable kG = new Tunable("Arm kG", 0.2, (val) -> {
        armFeedforward = new ArmFeedforward(armFeedforward.getKs(), val, armFeedforward.getKv());
    });

    public Tunable kV = new Tunable("Arm kV", 0, (val) -> {
        armFeedforward = new ArmFeedforward(armFeedforward.getKs(), armFeedforward.getKg(), val);
    });

    public Tunable kP = new Tunable("Arm kP", 130, (val) -> {
        pidController = new PIDController(val, pidController.getI(), pidController.getD());
    });

    public Tunable kI = new Tunable("Arm kI", 0, (val) -> {
        pidController = new PIDController(pidController.getP(), val, pidController.getD());
    });

    public Tunable kD = new Tunable("Arm kD", 5, (val) -> {
        pidController = new PIDController(pidController.getP(), pidController.getI(), val);
    });

    // double appliedVoltage = 0;

    public Arm() {
        TrapezoidProfile.Constraints constraints = new
        TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);
        armMotor = new SparkMax(29, MotorType.kBrushless);
                                                                                         // range in this case 1
                                                                                         // rotation
                                                                                         // per rotation, 3rd number is
                                                                                         // offset (whatever number you
                                                                                         // have to add so it reads zero
                                                                                         // degrees when horizontal)
        encoder = new DutyCycleEncoder(0, 1, 0); // 1st # is port, 2nd is ratio to rotations of
                                                                  // mechanism
                                                                  // (1 here), 3rd is initial offset (TODO to be
                                                                  // measured)
        // pidController = new PIDController(35, 0, 0);
        pidController.setTolerance(3 / 360.0, 1.0 / 360.0);
        pidController.enableContinuousInput(-0.5, 0.5);
        

        if (!encoder.isConnected()) {
            Elastic.sendNotification(new Notification(NotificationLevel.ERROR, "Encoder disconnected!",
                    "J2 arm encoder not connected!"));
        }

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg
                .smartCurrentLimit(50) // TODO find stall limit
                .idleMode(IdleMode.kBrake)
                .inverted(true)
                .voltageCompensation(12); // TODO needed?

        armMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armMotorSim = new SparkMaxSim(armMotor, DCMotor.getNEO(1));

        if (Robot.isSimulation()) {
            encoderSim = new DutyCycleEncoderSim(encoder);
            // encoderSim.set(0);
        }

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable armTable = inst.getTable("Arm");
        voltagePub = armTable.getDoubleTopic("Arm Voltage").publish();
        RotPub = armTable.getDoubleTopic("Arm Angle Degrees").publish();
        setpointPub = armTable.getDoubleTopic("PID Setpoint - Angle Desgrees").publish();
        percentRotPub = armTable.getDoubleTopic("Arm Percent Rotation").publish();
    }

    public boolean atSetPoint() {
        return pidController.atSetpoint();
    }

    public void stopMotor() {
        armMotor.stopMotor();
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(encoder.get());
    }

    @AutoLogOutput
    public double getPercentRotation() {

        double distance = PoseUtils.getAngleDistance(getAngle(), MIN_ANGLE).getDegrees();
        double interval = PoseUtils.getAngleDistance(MAX_ANGLE, MIN_ANGLE).getDegrees();
        return distance / interval;
    }

    public void setIdleMode(IdleMode mode) {
        armMotor.configure(new SparkMaxConfig().idleMode(mode), ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }
    public void setAngle(Rotation2d targetAngle) {
        double target = targetAngle.getRotations();

        // if (target > MAX_ANGLE.getRotations()) {
        //     Elastic.sendNotification(new Notification(
        //             NotificationLevel.WARNING, "Setting the arm angle outside of range",
        //             "Somebody is messing up by setting the angle to higher the range",
        //             5000));
        // }
        // if (target < MIN_ANGLE.getRotations()) {
        //     Elastic.sendNotification(new Notification(
        //             NotificationLevel.WARNING, "Setting the arm angle outside of range",
        //             "Somebody is messing up setting the angle",
        //             5000));
        // }

        // SmartDashboard.putNumber("Arm/preclamped Target", target);
        target = MathUtil.clamp(target, MIN_ANGLE.getRotations(), MAX_ANGLE.getRotations());
        // System.out.println("Target: " + target);
        // SmartDashboard.putNumber("Arm/Clamped Target", target);
        double wristVoltage = pidController.calculate(getAngle().getRotations(), target);
        Logger.recordOutput("Arm/PID Setpoint", target * 360);

        setpointPub.set(target * 360);
        
    

        //needs feedforward only when we have algae, because algae is heavy!
        // if(RobotContainer.rollers.hasAlgae()) 
        // if(getElbowAngle().getDegrees() > 0)
        // wristVoltage += -armFeedforward.calculate(getElbowAngle().getRadians(), 0);
        // else wristVoltage += armFeedforward.calculate(getElbowAngle().getRadians(), 0);
    
        wristVoltage = MathUtil.clamp(wristVoltage, -armLimitVoltage, armLimitVoltage);
        // System.out.println(-wristVoltage);
        // TODO do we need feedforward? If so we have to figure out the equation
        // negative voltage brings it up, positive brings it down AFAIK
        voltagePub.set(-wristVoltage);
        Logger.recordOutput("Arm/AppliedVoltage", -wristVoltage);
        armMotor.setVoltage(-wristVoltage);

    }

    public void joystickControl() {
        // don't invert joystick because we want up to apply a negative voltage
        double voltage = Math.pow(MathUtil.applyDeadband(RobotContainer.operatorController.getRightY(), 0.04), 3) * 12;
        // voltage = accelLimiter.calculate(voltage);

        double percentRot = getPercentRotation();

        if(getAngle().getDegrees() > 0)
            voltage += armFeedforward.calculate(getAngle().getRadians(), 0);
        else 
            voltage += -armFeedforward.calculate(getAngle().getRadians(), 0);

        if (voltage > 0) {
            if (percentRot <= 0.04) {
                voltage = 0;
                accelLimiter.reset(0);
            } else if (percentRot <= 0.07) {
                voltage = Math.max(voltage,
                        (-12 * Math.pow((percentRot * (100.0 / SLOW_DOWN_ZONE)), 3.2)) - SLOWEST_SPEED);
            }
        }
        if (voltage < 0) {
            if (percentRot >= 0.99) {
                voltage = 0;
                accelLimiter.reset(0);
            } else if (percentRot >= 0.93) {
                voltage = Math.min(voltage,
                        (12 * Math.pow((percentRot * (100.0 / SLOW_DOWN_ZONE)), 3.2)) + SLOWEST_SPEED);
            }
        }
        
        // double ffVoltage = armSim.feedforward(VecBuilder.fill(getElbowAngle().getRadians(), getWristAngle().getRadians())).get(0,0);
        // voltage = voltage - ffVol
        // if(RobotContainer.rollers.hasAlgae())
        //TODO check safeties after ff 

        voltage = MathUtil.clamp(voltage, -1, 1);
        // System.out.println(voltage);
        voltagePub.set(voltage);
        // voltage = MathUtil.clamp(voltage, -(12 * Math.pow((percentRot * (100.0 /
        // SLOW_DOWN_ZONE)), 3.0)
        // - SLOWEST_SPEED), /* lowest voltage allowed */
        // (12 * ((1 - percentRot) * (100.0 / SLOW_DOWN_ZONE))) + SLOWEST_SPEED) /*
        // highest voltage allowed */;
        // This clamps the voltage as it gets closer to the the top or the bottom. The
        // slow down zone is the area at the top or the bottom when things.
        // The slowest speed will allow the wrist to still go up and down no mater
        // what as long it has not hit the limit switch
        // Puting it to the power of 3 makes the slowdown more noticable
        // appliedVoltage = voltage;
        armMotor.setVoltage(voltage);
    }

    public void stop() {
        armMotor.stopMotor();
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> stopMotor());
    }

    @Override
    public void periodic() {
        if(RobotContainer.debugMode && !DriverStation.isFMSAttached()) {
            SmartDashboard.putBoolean("Arm/At Setpoint", atSetPoint());
            percentRotPub.set(getPercentRotation());
            // prevLoopTime = Timer.getFPGATimestamp();
            RotPub.set(getAngle().getDegrees());
        }
        Logger.recordOutput("Arm Degrees", getAngle().getDegrees());
        Logger.recordOutput("Arm/Encoder Connected", encoder.isConnected());
        Logger.recordOutput("Arm/Motor Applied Output", armMotor.getAppliedOutput());
    }

    @Override
    public void simulationPeriodic() {
        // Increment the simulation of the motor
        armMotorSim.iterate(armMotorSim.getAppliedOutput() * 2, RobotController.getBatteryVoltage(), 0.02);

    }

    public Command goToAngleCommand(Rotation2d angle) {
        return Commands.run(() -> setAngle(angle), this).until(this::atSetPoint).finallyDo(this::stopMotor);
    }

    public Command goToAngleContinuousCommand(Rotation2d angle) {
        return Commands.run(() -> setAngle(angle), this).finallyDo(this::stopMotor);
    }

    public Command joystickControlCommand() {
        return Commands.run(this::joystickControl, this);
    }

    public Command setIdleModeCommand(IdleMode mode) {
        return Commands.runOnce(() -> setIdleMode(mode)).ignoringDisable(true);
    }

    public Command setState(RobotState state) {
        return goToAngleCommand(state.getAngle());
    }

    public Command holdState(RobotState state) {
        return goToAngleContinuousCommand(state.getAngle());
    }

}
