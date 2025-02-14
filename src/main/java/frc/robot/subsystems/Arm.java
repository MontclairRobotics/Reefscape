package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.ArmPosition;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import frc.robot.util.simulation.DoubleJointedArmModel;

public class Arm extends SubsystemBase {
    public final double WRIST_SPEED = 0.5; //TODO set
    public final double MAX_VELOCITY = 0.5;
    public final double MAX_ACCELERATION = 0.5;
    private final ProfiledPIDController pidController;
    private static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(0); //The min safe angle of the endpoint to the horizontal //TODO set
    private static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0); //The max safe angle of the endpoint to the horizontal //TODO set
    private static final double LARGE_ANGLE_TO_SMALL = 30.0 / 14.0; //TODO check
    private static final double ENCODER_TO_LARGE_ANGLE = 12.0 / 18.0; // 12 teeth on gear near motor, 18 on gear near mechanism
    private static final double MOTOR_TO_ENCODER = -1; // TODO can't get gearbox from CAD
    private static final Rotation2d SMALL_ANGLE_WHEN_LARGE_IS_HORIZONTAL = Rotation2d.fromDegrees(-34.429); //TODO check

    private static final double J1Length = 0.19;
    private double j1PrevPos;
    private double j2PrevPos;
    private double j1Velocity;
    private double j2Velocity;
    private double prevLoopTime = Timer.getFPGATimestamp();
    private SlewRateLimiter accelLimiter = new SlewRateLimiter(8); // accelerate fully in ~1.5 seconds (can tune value)

    private DoubleJointedArmModel armSim;
    private final double SLOW_DOWN_ZONE = 7.0; //The percent at the top and bottom of the elevator out of the hight of the elevator extention in which the elevator will slow down to avoid crashing during manual control
    private final double SLOWEST_SPEED = 0.3;

    private DutyCycleEncoder j1Encoder;
    // TODO will there be 2 encoders?
    private DutyCycleEncoder j2Encoder;

    DutyCycleEncoderSim j1EncoderSim;
    DutyCycleEncoderSim j2EncoderSim;
    private SparkMax armMotor;
    private SparkMaxSim wristSim;
    public double smallWristAngle;
    public double largeWristAngle;

    private DoublePublisher voltagePub;
    private DoublePublisher largeRotPub;
    private DoublePublisher smallRotPub;

    private DoublePublisher setpointPub;

    private StructPublisher<Pose3d> joint1PosePub;
    private StructPublisher<Pose3d> joint2PosePub;

    // SIM
    private DoublePublisher simJ1EncoderPub;
    private DoublePublisher simJ2EncoderPub;

    public Arm() {
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);
        armMotor = new SparkMax(-1, MotorType.kBrushless);
        j1Encoder = new DutyCycleEncoder(0, 1 * ENCODER_TO_LARGE_ANGLE, -68.8 / 360); // 1st number is port, 2nd is range in this case 1 rotation per rotation, 3rd number is offset (whatever number you have to add so it reads zero degrees when horizontal)
        j2Encoder = new DutyCycleEncoder(1, 1, -34.429 / 360); // 1st # is port, 2nd is ratio to rotations of mechanism (1 here), 3rd is initial offset (TODO to be measured)
        pidController = new ProfiledPIDController(30.0, 0.0, 0.0, constraints);
        pidController.setTolerance(1.0 / 360.0);
        
        if (!j1Encoder.isConnected()) {
            Elastic.sendNotification(new Notification(NotificationLevel.ERROR, "Encoder disconnected!", "J1 arm encoder not connected!"));
        }

        if (!j2Encoder.isConnected()) {
            Elastic.sendNotification(new Notification(NotificationLevel.ERROR, "Encoder disconnected!", "J2 arm encoder not connected!"));
        }

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg
            .smartCurrentLimit(50) //TODO find stall limit
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .voltageCompensation(12); //TODO needed?
        
        armMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        

        wristSim = new SparkMaxSim(armMotor, DCMotor.getNEO(1));
        // armSim = new DoubleJointedArmModel(2, 0.25, 0.5, 0.5, DCMotor.getNEO(1).withReduction(ENCODER_TO_LARGE_ANGLE), 6, 0.25, 0.5, DCMotor.getNEO(1).withReduction(ENCODER_TO_LARGE_ANGLE), 1); //TODO how to handle 2nd motor?

        if (Robot.isSimulation()) {
            j1EncoderSim = new DutyCycleEncoderSim(j1Encoder);
            j2EncoderSim = new DutyCycleEncoderSim(j2Encoder);
            j1EncoderSim.set(0);
            j2EncoderSim.set(SMALL_ANGLE_WHEN_LARGE_IS_HORIZONTAL.getRotations());
            // j2EncoderSim.set(0);
        }

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable armTable = inst.getTable("Arm");
        voltagePub = armTable.getDoubleTopic("Arm Voltage").publish();
        largeRotPub = armTable.getDoubleTopic("Large Arm Angle Degrees").publish();
        smallRotPub = armTable.getDoubleTopic("Small Arm Angle Degrees").publish();
        setpointPub = armTable.getDoubleTopic("PID Setpoint - Small Angle Desgrees").publish();
        NetworkTable simTable = armTable.getSubTable("Sim");

        joint1PosePub = armTable.getStructTopic("Joint1Pose", Pose3d.struct).publish();
        joint2PosePub = armTable.getStructTopic("Joint2Pose", Pose3d.struct).publish();

        simJ1EncoderPub = simTable.getDoubleTopic("Joint 1 Sim Angle").publish();
        simJ2EncoderPub = simTable.getDoubleTopic("Joint 2 Sim Angle").publish();
    }

    public boolean atSetpoint() {
        return pidController.atGoal();
    }

    public void stopMotor() {
        armMotor.stopMotor();
    }

    public void setIdleMode(IdleMode mode) {
        armMotor.configure(new SparkMaxConfig().idleMode(mode), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /*
     * Returns the angle to the horizontal of the endpoint of the arm in rotations
     */
    public Rotation2d getEndpointAngle() {
        // return Rotation2d.fromRotations(j1Encoder.get() - ((j1Encoder.get() * LARGE_ANGLE_TO_SMALL) + SMALL_ANGLE_WHEN_LARGE_IS_HORIZONTAL.getRotations()));
        // OR
        return Rotation2d.fromRotations(smallWristAngle); //TODO check? should be subtract?
    }

    private void setWristAngle(Rotation2d targetAngle) {
        double target = targetAngle.getRotations();
        System.out.println(targetAngle.getDegrees());
        System.out.println(getEndpointAngle().getRotations() * 360);
        setpointPub.set(target * 360);
        if (target > MAX_ANGLE.getRotations()) {
            Elastic.sendNotification(new Notification(
                    NotificationLevel.WARNING, "Setting the elevator height outside of range",
                    "Somebody is messing up the button setting in robot container by setting the height to higher the range",
                    5000));
        }
        if (target < MAX_ANGLE.getRotations()) {
            Elastic.sendNotification(new Notification(
                    NotificationLevel.WARNING, "Setting the elevator height outside of range",
                    "Somebody is messing up the button setting in robot container by setting the height to lower than 0 (who is doing this???! WTF??)",
                    5000));
        }
        
        // target = MathUtil.clamp(target, MIN_ANGLE.getRotations(), MAX_ANGLE.getRotations());
        double wristVoltage = pidController.calculate(getEndpointAngle().getRotations(), target);

        wristVoltage = MathUtil.clamp(wristVoltage, -12, 12);
        // TODO do we need feedforward? If so we have to figure out the equation
        armMotor.setVoltage(wristVoltage);
    }

    public void setWristLocation(ArmPosition pos) {
        setWristAngle(pos.getAngle());
    }

    public void joystickControl() {
        double voltage = Math.pow(MathUtil.applyDeadband(RobotContainer.operatorController.getRightY(), 0.04), 3) * 12;
        // voltage = accelLimiter.calculate(voltage);

        double percentRot = getEndpointAngle().getRotations() / (MAX_ANGLE.getRotations() - MIN_ANGLE.getRotations());
        // System.out.println(voltage);
        // System.out.println("Percent Height: " + percentHeight);
        // System.out.println("Elevator ff Voltage: " + elevatorFeedforward.calculate(0));

        // if (voltage < 0) {
        //     if (percentRot <= 0.004) {
        //         voltage = 0;
        //         accelLimiter.reset(0);
        //     } else if (percentRot <= 0.07) {
        //         voltage = Math.max(voltage, (-12 * Math.pow((percentRot * (100.0 / SLOW_DOWN_ZONE)), 3.2)) - SLOWEST_SPEED);
        //     }
        // }
        // if (voltage > 0) {
        //     if (percentRot >= 0.996) {
        //         voltage = 0;
        //         accelLimiter.reset(0);
        //     } else if (percentRot >= 0.93) {
        //         voltage = Math.min(voltage, (12 * Math.pow((percentRot * (100.0 / SLOW_DOWN_ZONE)), 3.2)) + SLOWEST_SPEED);
        //     }
        // }
        voltage = MathUtil.clamp(voltage, -12, 12);
        voltagePub.set(voltage);

        // voltage = MathUtil.clamp(voltage, -(12 * Math.pow((percentHeight * (100.0 / SLOW_DOWN_ZONE)), 3.0)
        //         - SLOWEST_SPEED), /* lowest voltage allowed */
        //         (12 * ((1 - percentHeight) * (100.0 / SLOW_DOWN_ZONE))) + SLOWEST_SPEED) /* highest voltage allowed */;
        // This clamps the voltage as it gets closer to the the top or the bottom. The
        // slow down zone is the area at the top or the bottom when things.
        // The slowest speed will allow the wrist to still go up and down no mater
        // what as long it has not hit the limit switch
        // Puting it to the power of 3 makes the slowdown more noticable
        armMotor.setVoltage(voltage);
    }

    public void stop() {
        armMotor.stopMotor();;
    }

    @Override
    public void periodic() {
        largeWristAngle = j1Encoder.get();
        smallWristAngle = j2Encoder.get(); // OR (largeWristAngle * LARGE_ANGLE_TO_SMALL) + SMALL_ANGLE_WHEN_LARGE_IS_HORIZONTAL.getRotations(); //TODO is this 30 right? I dont think so


        if (Robot.isSimulation()) {
            largeWristAngle = j1EncoderSim.get();
            smallWristAngle = j2EncoderSim.get();
        }

        j1Velocity = (largeWristAngle - j1PrevPos) / 0.02;
        j2Velocity = (smallWristAngle - j2PrevPos) / 0.02;
        prevLoopTime = Timer.getFPGATimestamp();
        j1PrevPos = largeWristAngle;
        j2PrevPos = smallWristAngle;

        largeRotPub.set(largeWristAngle * 360);
        smallRotPub.set(smallWristAngle * 360);
    }

    @Override
    public void simulationPeriodic() {
        // double dt = 0.02; //Timer.getFPGATimestamp() - prevLoopTime;
        // Matrix<N2, N2> stateMatrix = new Matrix<N2, N2>(N2.instance, N2.instance);
        // stateMatrix.set(0, 0, largeWristAngle * 2 * Math.PI);
        // stateMatrix.set(1, 0, smallWristAngle * 2 * Math.PI);
        // stateMatrix.set(0, 1, j1Velocity * 2 * Math.PI);
        // stateMatrix.set(1, 1, j2Velocity * 2 * Math.PI);
        // double voltage = wristSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        // double voltage2 = MathUtil.clamp(voltage * LARGE_ANGLE_TO_SMALL, -12, 12);
        // System.out.println(voltage);
        // Matrix<N2, N2> nextStates = armSim.simulate(stateMatrix, VecBuilder.fill(voltage, 0), dt);

        // // System.out.println(nextStates.get(0,0));
        // wristSim.iterate(Units.radiansPerSecondToRotationsPerMinute(nextStates.get(0, 1)), RobotController.getBatteryVoltage(), dt);
        // j1EncoderSim.set(Units.radiansToRotations(nextStates.get(0, 0)));
        // j2EncoderSim.set(Units.radiansToRotations(nextStates.get(1, 0)));

        wristSim.iterate(wristSim.getAppliedOutput() * MAX_VELOCITY, RobotController.getBatteryVoltage(), 0.02);
        // System.out.println(largeWristAngle);
        j1EncoderSim.set(((largeWristAngle + (wristSim.getAppliedOutput() * MAX_VELOCITY) * 0.02) + 1)% 1);
        j2EncoderSim.set(((smallWristAngle + ( -wristSim.getAppliedOutput() * MAX_VELOCITY * (30.0/14.0)) * 0.02) + 1) % 1);
        simJ1EncoderPub.set(j1EncoderSim.get() * 360);
        simJ2EncoderPub.set(j2EncoderSim.get() * 360);

        double largeVisualizationAngle = (largeWristAngle + 0.1047);
        double smallVisualizationAngle = 1 - (smallWristAngle) + (-78.0/360.0) + 0.1047);

        joint1PosePub.set(new Pose3d(0.121, 0, RobotContainer.elevator.getHeight() - Units.inchesToMeters(1.5), new Rotation3d(0, largeVisualizationAngle * 2 * Math.PI, 0)));

        double j2X = (J1Length - Units.inchesToMeters(3)) * Math.cos(largeWristAngle * 2 * Math.PI);
        double j2Z = (J1Length - Units.inchesToMeters(3))* -Math.sin(largeWristAngle * 2 * Math.PI);
        joint2PosePub.set(new Pose3d(j2X + 0.121, 0, RobotContainer.elevator.getHeight() + j2Z  - Units.inchesToMeters(1.5), new Rotation3d(0, (largeVisualizationAngle - smallVisualizationAngle) * 2 * Math.PI, 0)));
    }

    public Command goToAngleCommand(Rotation2d angle) {
        return Commands.run(() -> setWristAngle(angle), this).until(pidController::atGoal).finallyDo(this::stop);
    }

    public Command joystickControlCommand() {
        return Commands.run(this::joystickControl, this);
    }

    public Command setIdleModeCommand(IdleMode mode) {
        return Commands.runOnce(() -> setIdleMode(mode)).ignoringDisable(true);
    }

    public Command goToLocationCommand(ArmPosition pos) {
        return goToAngleCommand(pos.getAngle());
    }

}
