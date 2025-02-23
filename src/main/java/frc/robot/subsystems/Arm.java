package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Rotations;

import java.util.GregorianCalendar;

import org.dyn4j.geometry.Rotation;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
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

    public final double MAX_VELOCITY = 60.0 / 360.0; // rotations per sec
    public final double MAX_ACCELERATION = 20.0 / 360.0; // rotations per sec per sec
    
    private static final Rotation2d ELBOW_ENCODER_OFFSET = Rotation2d.fromDegrees(-83);
    private static final Rotation2d ELBOW_MAX_ANGLE = Rotation2d.fromDegrees(34); //TODO: use protractor to get this for the real robot
    private static final Rotation2d ELBOW_MIN_ANGLE = Rotation2d.fromDegrees(-56); //TODO: use protractor to get this for the real robot

    // The max safe angle of the endpoint to the horizontal 
    private static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(36.8); 
                                                                              
    // The min safe angle of the endpoint to the horizontal
    private static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-(180 - 102.143)); 

    // Angle of endpoint is -37.8
    private static final double ELBOW_ANGLE_TO_WRIST = 30.0 / 14.0; // TODO check

    // TODO: grab value from real robot using protractor
    private static final Rotation2d WRIST_ANGLE_WHEN_ELBOW_IS_HORIZONTAL = Rotation2d.fromDegrees(-34.903); 

    private ArmFeedforward armFeedforward = new ArmFeedforward(0, 0.2, 0); 

    private PIDController pidController = new PIDController(17, 0, 0);


    private static final double J1Length = 0.19 - Units.inchesToMeters(2);
    private double j1PrevPos;
    private double j2PrevPos;
    private double j1Velocity = 0;
    private double j2Velocity = 0;
    private double prevLoopTime = Timer.getFPGATimestamp();
    
    private SlewRateLimiter accelLimiter = new SlewRateLimiter(8); // accelerate fully in ~1.5 seconds (can tune value)

    private DoubleJointedArmModel armSim;
    private final double SLOW_DOWN_ZONE = 7.0; // The percent at the top and bottom of the elevator out of the hight of
                                               // the elevator extention in which the elevator will slow down to avoid
                                               // crashing during manual control
    private final double SLOWEST_SPEED = 0.3;

    private DutyCycleEncoder elbowEncoder;
    // TODO will there be 2 encoders?
    private DutyCycleEncoder wristEncoder;

    DutyCycleEncoderSim elbowEncoderSim;
    DutyCycleEncoderSim wristEncoderSim;
    private SparkMax armMotor;
    private SparkMaxSim armMotorSim;
    // public double smallWristAngle;
    // public double largeWristAngle;

    private DoublePublisher voltagePub;
    private DoublePublisher largeRotPub;
    private DoublePublisher smallRotPub;

    private DoublePublisher setpointPub;
    private DoublePublisher endPointAnglePub;
    private DoublePublisher percentRotPub;

    private StructPublisher<Pose3d> elbowPosePub;
    private StructPublisher<Pose3d> wristPosePub;

    public Tunable kG = new Tunable("Arm kG", 0.2, (val) -> {
        armFeedforward = new ArmFeedforward(armFeedforward.getKs(), val, armFeedforward.getKv());
    });

    public Tunable kV = new Tunable("Arm kV", 0, (val) -> {
        armFeedforward = new ArmFeedforward(armFeedforward.getKs(), armFeedforward.getKg(), val);
    });

    public Tunable kP = new Tunable("Arm kP", 17, (val) -> {
        pidController = new PIDController(val, pidController.getI(), pidController.getD());
    });

    public Tunable kI = new Tunable("Arm kI", 0, (val) -> {
        pidController = new PIDController(pidController.getP(), val, pidController.getD());
    });

    public Tunable kD = new Tunable("Arm kD", 0, (val) -> {
        pidController = new PIDController(pidController.getP(), pidController.getI(), val);
    });

    // double appliedVoltage = 0;

    public Arm() {
        TrapezoidProfile.Constraints constraints = new
        TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);
        armMotor = new SparkMax(29, MotorType.kBrushless);
        elbowEncoder = new DutyCycleEncoder(0, 1 , ELBOW_ENCODER_OFFSET.getRotations());
        elbowEncoder.setInverted(true);
         // 1st number is port, 2nd is
                                                                                         // range in this case 1
                                                                                         // rotation
                                                                                         // per rotation, 3rd number is
                                                                                         // offset (whatever number you
                                                                                         // have to add so it reads zero
                                                                                         // degrees when horizontal)
        wristEncoder = new DutyCycleEncoder(1, 1, 0); // 1st # is port, 2nd is ratio to rotations of
                                                                  // mechanism
                                                                  // (1 here), 3rd is initial offset (TODO to be
                                                                  // measured)
        // pidController = new PIDController(35, 0, 0);
        pidController.setTolerance(0.5 / 360.0);
        pidController.enableContinuousInput(-0.5, 0.5);

        if (!elbowEncoder.isConnected()) {
            Elastic.sendNotification(new Notification(NotificationLevel.ERROR, "Encoder disconnected!",
                    "J1 arm encoder not connected!"));
        }

        // if (!wristEncoder.isConnected()) {
        //     Elastic.sendNotification(new Notification(NotificationLevel.ERROR, "Encoder disconnected!",
        //             "J2 arm encoder not connected!"));
        // }

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg
                .smartCurrentLimit(50) // TODO find stall limit
                .idleMode(IdleMode.kBrake)
                .inverted(true)
                .voltageCompensation(12); // TODO needed?

        armMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armMotorSim = new SparkMaxSim(armMotor, DCMotor.getNEO(1));
        armSim = new DoubleJointedArmModel(2, 0.25, 1.5, 0.5, DCMotor.getNEO(1),
                6, 0.25, 0.5, DCMotor.getNEO(1), 0.001); // TODO how to handle 2nd,
                                                                                           // motor?

        if (Robot.isSimulation()) {
            elbowEncoderSim = new DutyCycleEncoderSim(elbowEncoder);
            wristEncoderSim = new DutyCycleEncoderSim(wristEncoder);
            elbowEncoderSim.set(0);
            // wristEncoderSim.set(WRIST_ANGLE_WHEN_ELBOW_IS_HORIZONTAL.getRotations());
            // wristEncoderSim.set(0);
            // j2EncoderSim.set(0);
        }

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable armTable = inst.getTable("Arm");
        voltagePub = armTable.getDoubleTopic("Arm Voltage").publish();
        largeRotPub = armTable.getDoubleTopic("Large Arm Angle Degrees").publish();
        smallRotPub = armTable.getDoubleTopic("Small Arm Angle Degrees").publish();
        setpointPub = armTable.getDoubleTopic("PID Setpoint - Small Angle Desgrees").publish();
        endPointAnglePub = armTable.getDoubleTopic("Endpoint Degrees").publish();
        percentRotPub = armTable.getDoubleTopic("Arm Percent Rotation").publish();

        elbowPosePub = armTable.getStructTopic("Joint1Pose", Pose3d.struct).publish();
        wristPosePub = armTable.getStructTopic("Joint2Pose", Pose3d.struct).publish();
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    public void stopMotor() {
        armMotor.stopMotor();
    }

    public double getPercentRotation() {

        double max = 32;
        double min = -56;
      //  double distance = PoseUtils.getAngleDistance(getEndpointAngle(), MIN_ANGLE).getDegrees();
       // double interval = PoseUtils.getAngleDistance(MAX_ANGLE, MIN_ANGLE).getDegrees();
        // interval = MAX_ANGLE.getDegrees() - MIN_ANGLE.getDegrees();
        // distance = getEndpointAngle().getDegrees() + MIN_ANGLE.getDegrees();
         double interval = max - min;
         double distance = getElbowAngle().getDegrees() - min;
        return distance / interval;
    }

    public void setIdleMode(IdleMode mode) {
        armMotor.configure(new SparkMaxConfig().idleMode(mode), ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }


    /**
     * Returns the angle of the elbow to the horizontal
     */
    public Rotation2d getElbowAngle() {
        return Drivetrain.wrapAngle(Rotation2d.fromRotations(elbowEncoder.get()));
    }

    /**
     * Returns the angle from the elbow to the wrist
     */
    public Rotation2d getWristAngle() {
        // return Rotation2d.fromRotations(wristEncoder.get());
        if (Robot.isReal()) {
         return Rotation2d.fromRotations((getElbowAngle().getRotations() * (-ELBOW_ANGLE_TO_WRIST)) + WRIST_ANGLE_WHEN_ELBOW_IS_HORIZONTAL.getRotations());
        }
        return Rotation2d.fromRotations(wristEncoder.get());
    }

    /*
     * Returns the angle to the horizontal of the endpoint of the arm in rotations
     */
    public Rotation2d getEndpointAngle() {
        // return Rotation2d.fromRotations(j1Encoder.get() - ((j1Encoder.get() *
        // LARGE_ANGLE_TO_SMALL) +
        // SMALL_ANGLE_WHEN_LARGE_IS_HORIZONTAL.getRotations()));
        // OR
        // return PoseUtils.wrapRotation(Rotation2d.fromRotations((smallWristAngle +
        // largeWristAngle))); // TODO check?
        return getElbowAngle().plus(getWristAngle());
    }

    private void setEndpointAngle(Rotation2d targetAngle) {
        double target = targetAngle.getRotations();

        setpointPub.set(target * 360);
        if (target > MAX_ANGLE.getRotations()) {
            Elastic.sendNotification(new Notification(
                    NotificationLevel.WARNING, "Setting the elevator height outside of range",
                    "Somebody is messing up the button setting in robot container by setting the height to higher the range",
                    5000));
        }
        if (target < MIN_ANGLE.getRotations()) {
            Elastic.sendNotification(new Notification(
                    NotificationLevel.WARNING, "Setting the elevator height outside of range",
                    "Somebody is messing up the button setting in robot container by setting the height to lower than 0 (who is doing this???! WTF??)",
                    5000));
        }

        target = MathUtil.clamp(target, MIN_ANGLE.getRotations(), MAX_ANGLE.getRotations());
        double wristVoltage = pidController.calculate(getEndpointAngle().getRotations(), target);

        //needs feedforward only when we have algae, because algae is heavy!
        // if(RobotContainer.rollers.hasAlgae()) 
        // if(getElbowAngle().getDegrees() > 0)
        // wristVoltage += -armFeedforward.calculate(getElbowAngle().getRadians(), 0);
        // else wristVoltage += armFeedforward.calculate(getElbowAngle().getRadians(), 0);
    
        wristVoltage = MathUtil.clamp(wristVoltage, -3, 3);
        System.out.println(-wristVoltage);
        // TODO do we need feedforward? If so we have to figure out the equation
        // negative voltage brings it up, positive brings it down AFAIK
        armMotor.setVoltage(-wristVoltage);

    }

    public void joystickControl() {
        // don't invert joystick because we want up to apply a negative voltage
        double voltage = Math.pow(MathUtil.applyDeadband(RobotContainer.operatorController.getRightY(), 0.04), 3) * 12;
        // voltage = accelLimiter.calculate(voltage);

        double percentRot = getPercentRotation();

        //percentRot is based on endpoint rotation, which moves in the opposite direction as the motor
        if (voltage < 0) {
            if (percentRot <= 0.02) {
                voltage = 0;
                accelLimiter.reset(0);
            } else if (percentRot <= 0.07) {
                voltage = Math.max(voltage,
                        (-12 * Math.pow((percentRot * (100.0 / SLOW_DOWN_ZONE)), 3.2)) - SLOWEST_SPEED);
            }
        }
        if (voltage > 0) {
            if (percentRot >= 0.98) {
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
        if(getElbowAngle().getDegrees() > 0)
        voltage += armFeedforward.calculate(getElbowAngle().getRadians(), 0);
        else voltage += -armFeedforward.calculate(getElbowAngle().getRadians(), 0);
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
        percentRotPub.set(getPercentRotation());

        // These lines are for an actual physics simulator
        // j1Velocity = (getElbowAngle().getRotations() - j1PrevPos) / 0.02;
        // j1Velocity = 0;
        // j2Velocity = (getWristAngle().getRotations() - j2PrevPos) / 0.02;
        // j2Velocity = 0;
        // prevLoopTime = Timer.getFPGATimestamp();
        j1PrevPos = getElbowAngle().getRotations();
        j2PrevPos = getWristAngle().getRotations();

        largeRotPub.set(getElbowAngle().getDegrees());
        smallRotPub.set(getWristAngle().getDegrees());
        endPointAnglePub.set(getEndpointAngle().getDegrees());
    }

    @Override
    public void simulationPeriodic() {
        // double dt = 0.02; //Timer.getFPGATimestamp() - prevLoopTime;
        // Matrix<N2, N2> stateMatrix = new Matrix<N2, N2>(N2.instance, N2.instance);
        // stateMatrix.set(0, 0, -getElbowAngle().getRadians());
        // stateMatrix.set(1, 0, getWristAngle().getRadians());
        // // double voltage = appliedVoltage;
        // double voltage = Math.pow(-MathUtil.applyDeadband(RobotContainer.operatorController.getRightY(), 0.04), 3) * 12;
        // // double voltage2 = MathUtil.clamp((voltage * -30.0 / 14.0), -12, 12);
        // double voltage2 = 0;
        // // System.out.println(voltage);

        // stateMatrix.set(0, 1, j1Velocity);
        // stateMatrix.set(1, 1, j2Velocity);
        // // double voltage = armMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        // // System.out.println(voltage);
        // // if (true) {
        // // System.out.println(voltage);
        // Matrix<N2, N2> nextStates = armSim.simulate(stateMatrix,
        // VecBuilder.fill(voltage, voltage2), dt);

        // SmartDashboard.putNumber("J1 Velocity", j1Velocity);
        // SmartDashboard.putNumber("Modified J2 Velocity", j2Velocity * (14.0 /30.0));


        // // armMotorSim.iterate(Units.radiansPerSecondToRotationsPerMinute(nextStates.get(0,1) / ENCODER_TO_ELBOW), RobotController.getBatteryVoltage(), dt);
        // j1Velocity = nextStates.get(0, 1);
        // // j2Velocity = nextStates.get(1, 1);
        // j2Velocity = nextStates.get(1, 1);//j1Velocity * (-30.0 / 14.0);
        // System.out.println(j1Velocity * (180.0 / Math.PI));

        // // System.out.println(nextStates.get(0,0));
        // // System.out.println(nextStates.get(0,1));
        
        // elbowEncoderSim.set(Units.radiansToRotations(nextStates.get(0, 0)) % 1);
        // wristEncoderSim.set((getElbowAngle().getRotations() * (-ELBOW_ANGLE_TO_WRIST)) + WRIST_ANGLE_WHEN_ELBOW_IS_HORIZONTAL.getRotations());
        // // wristEncoderSim.set(Units.radiansToRotations(nextStates.get(1, 0)) % 1);
        // // }

        // Increment the simulation of the motor
        armMotorSim.iterate(armMotorSim.getAppliedOutput() * MAX_VELOCITY, RobotController.getBatteryVoltage(), 0.02);

        // Because I don't feel like doing physics, assume each joint is moving at a
        // constant velocity

        // Mod by 1 because it's in rotations to keep angles wrapped between 0 and 360
        // .getAppliedOutput() is between 0 and 1, as a fraction of motor's output
        // multiple the velocity by 0.02 to get the change in position over 1 loop (0.02
        // seconds)
        // add one to ensure the rotations are positive. The plus 1 and mod 1 basically
        // ensure
        // that the encoders always read between 0 and 1 rotation (0 and 360 degrees)

        elbowEncoderSim.set(
                ((getElbowAngle().getRotations() + (armMotor.getAppliedOutput() * MAX_VELOCITY) * 0.02)) % 1);
        wristEncoderSim.set(
                ((getWristAngle().getRotations()
                        - (armMotor.getAppliedOutput() * MAX_VELOCITY * ELBOW_ANGLE_TO_WRIST) * 0.02)) % 1);

        // Publish sim encoder positions to the network

        // invert the angle for visualization because positive angle is down in
        // AdvantageScope
        // the constant added is because the CAD was exported on an angle (I think?) It
        // Just brings the simulated arm piece to the horizontal when the encoder reads
        // 0.
        double largeVisualizationAngle = -getElbowAngle().getRadians();

        // Invert the largeWristAngle and smallWristAngle because positive angle is down
        // in AdvantageScope
        double smallVisualizationAngle = -getEndpointAngle().getRadians();

        // Publish the pose for joint 1. The x coordinate is the coordinate of the
        // elevator
        // Y coordinate is zero because it is centered on that axis.
        elbowPosePub.set(new Pose3d(0.121, 0, RobotContainer.elevator.getExtension() + 0.937,
                new Rotation3d(0, largeVisualizationAngle, 0)));

        // Calculate position of second joint based on first joint length
        double xOffset = J1Length * Math.cos(getElbowAngle().getRadians());
        double yOffset = J1Length * Math.sin(getElbowAngle().getRadians());

        // Publish the pose for joint 2. The x coordinate is the coordinate of the
        // elevator plus
        // the length of the first part of the arm along the x axis. The y coordinate is
        // zero
        // because it is centered on that axis
        // The z coordinate is the elevator height plus the length of the first part of
        // the arm along the y axis
        wristPosePub
                .set(new Pose3d(xOffset + 0.121, 0, RobotContainer.elevator.getExtension() + yOffset + 0.937,
                        new Rotation3d(0, smallVisualizationAngle, 0)));
    }

    public Command goToAngleCommand(Rotation2d angle) {
        return Commands.run(() -> setEndpointAngle(angle), this).until(this::atSetpoint).finallyDo(this::stop);
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

}
