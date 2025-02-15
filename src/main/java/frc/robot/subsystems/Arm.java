package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import frc.robot.util.simulation.VariableSingleJointedArmSim;

public class Arm extends SubsystemBase {
    public final int MOTOR_ID = 25;
    public final int ELBOW_ENCODER_ID = 26;
    public final int WRIST_ENCODER_ID = 27;

    // public final double WRIST_SPEED = 0.5; //TODO set
    public final double MAX_VELOCITY = 15;
    public final double MAX_ACCELERATION = 15;
    private final ProfiledPIDController pidController;
    // private final PIDController pidController;
    private static final Rotation2d MAX_ELBOW_ANGLE = Rotation2d.fromDegrees(38); //The min safe angle of the endpoint to the horizontal //TODO set
    private static final Rotation2d MIN_ELBOW_ANGLE = Rotation2d.fromDegrees(-63); //The max safe angle of the endpoint to the horizontal //TODO set
    private static final Rotation2d STARTING_ELBOW_ANGLE = Rotation2d.fromDegrees(38); //The starting angle of the elbow //TODO set
    private static final double ELBOW_TO_WRIST_RATIO = 14.0 / 30.0; //TODO check
    private static final double ENCODER_TO_ELBOW_RATIO = 12.0 / 18.0; // 12 teeth on gear near motor, 18 on gear near mechanism
    private static final double MOTOR_TO_ENCODER = 50; // TODO can't get gearbox from CAD
    private static final Rotation2d SMALL_ANGLE_WHEN_LARGE_IS_HORIZONTAL = Rotation2d.fromDegrees(-34.429); //TODO check
    private SlewRateLimiter accelLimiter = new SlewRateLimiter(8); // accelerate fully in ~1.5 seconds (can tune value)

    private static final double FOREARM_LENGTH = Units.inchesToMeters(5); // From pivot to pivot  TODO: What is this in real life?
    private static final double HAND_LENGTH = Units.inchesToMeters(6); // From pivot to COM  TODO: What is this in real life?
    private static final double FOREARM_MASS = Units.lbsToKilograms(1); // TODO: What is this in real life?
    private static final double HAND_MASS = Units.lbsToKilograms(4); // TODO: What is this in real life?

    private final double SLOW_DOWN_ZONE = 7.0; //The percent at the top and bottom of the elevator out of the hight of the elevator extention in which the elevator will slow down to avoid crashing during manual control
    private final double SLOWEST_SPEED = 0.3;

    // This measures angle of forearm to horizontal
    private DutyCycleEncoder elbowEncoder;
    // This measures angle of hand to horizontal
    private DutyCycleEncoder wristEncoder;
    private SparkMax elbowMotor;
    private double outputVoltage = 0;


    private DoublePublisher voltagePub;
    private DoublePublisher elbowDegreesPub;
    private DoublePublisher wristDegreesPub;
    private DoublePublisher endpointDegreesPub;
    private DoublePublisher endpointSetpointPub;
    private DoublePublisher handEffectiveLengthPub;
    private DoublePublisher armEffectiveLengthPub;
    private DoublePublisher feedforwardPub;
    private DoublePublisher armCOGPub;
    private DoublePublisher handCOGPub;

    private StructPublisher<Pose3d> forearmPosePub;
    private StructPublisher<Pose3d> handPosePub;

    private VariableSingleJointedArmSim armSim;
    private DutyCycleEncoderSim elbowEncoderSim;
    private DutyCycleEncoderSim wristEncoderSim;
    


    public Arm() {
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);
        elbowMotor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
        elbowEncoder = new DutyCycleEncoder(ELBOW_ENCODER_ID, 1 * ENCODER_TO_ELBOW_RATIO, 0); // 1st number is port, 2nd is range in this case 1 rotation per rotation, 3rd number is offset (whatever number you have to add so it reads zero degrees when horizontal)
        wristEncoder = new DutyCycleEncoder(WRIST_ENCODER_ID, 1, 0); // 1st # is port, 2nd is ratio to rotations of mechanism (1 here), 3rd is initial offset (TODO to be measured)
        pidController = new ProfiledPIDController(200, 0, 0., constraints);
        // pidController = new PIDController(20, 0, 0.5);
        
        if (!elbowEncoder.isConnected()) {
            Elastic.sendNotification(new Notification(NotificationLevel.ERROR, "Encoder disconnected!", "Elbow encoder not connected!"));
        }

        if (!wristEncoder.isConnected()) {
            Elastic.sendNotification(new Notification(NotificationLevel.ERROR, "Encoder disconnected!", "Wrist encoder not connected!"));
        }

        SparkMaxConfig cfg = new SparkMaxConfig();        cfg
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .voltageCompensation(12); //TODO needed?
        
        elbowMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable armTable = inst.getTable("Arm");
        voltagePub = armTable.getDoubleTopic("Voltage").publish();
        elbowDegreesPub = armTable.getDoubleTopic("Elbow Degrees").publish();
        wristDegreesPub = armTable.getDoubleTopic("Wrist Degrees").publish();
        endpointDegreesPub = armTable.getDoubleTopic("EndPoint Degrees").publish();
        endpointSetpointPub = armTable.getDoubleTopic("EndPoint Setpoint Degrees").publish();

        forearmPosePub = armTable.getStructTopic("Forearm Pose", Pose3d.struct).publish();
        handPosePub = armTable.getStructTopic("Hand Pose", Pose3d.struct).publish();

        handEffectiveLengthPub = armTable.getDoubleTopic("Hand Effective Length").publish();
        armEffectiveLengthPub = armTable.getDoubleTopic("Arm Effective Length").publish();

        feedforwardPub = armTable.getDoubleTopic("Feed Forward").publish();

        armCOGPub = armTable.getDoubleTopic("Arm COG").publish();
        handCOGPub = armTable.getDoubleTopic("Hand COG").publish();

        if (Robot.isSimulation()) {
            armSim = new VariableSingleJointedArmSim(
                DCMotor.getNEO(1),
                MOTOR_TO_ENCODER * ENCODER_TO_ELBOW_RATIO, 
                0.3810168288,
                () -> getEffectiveLength(),
                MIN_ELBOW_ANGLE.getRadians(),
                MAX_ELBOW_ANGLE.getRadians(), 
                true, 
                STARTING_ELBOW_ANGLE.getRadians(),
                new double[] { 0.0, 0.0 });
            elbowEncoderSim = new DutyCycleEncoderSim(elbowEncoder);
            wristEncoderSim = new DutyCycleEncoderSim(wristEncoder);
        }
    }


    /*
     * Returns angle of forearm to horizontal
     */
    public Rotation2d getElbowAngle() {
        return Rotation2d.fromRotations(elbowEncoder.get());
    }
    /**
     * Returns angle of wrist to foream
     */
    public Rotation2d getWristAngle() {
        return Rotation2d.fromRotations(wristEncoder.get());
    }
    /*
     * Returns angle of wrist to horizontal
     */
    public Rotation2d getEndpointAngle() {
        // These three statements should all be the same or something slipped
        // elbowRotations = - wristRotations * ELBOW_TO_WRIST_RATIO

        // return getElbowRotations() + -getElbowRotations() / ELBOW_TO_WRIST_RATIO;
        // return getWristRotations() * -ELBOW_TO_WRIST_RATIO + getWristRotations();

        // These are added since the are opposite of each other
        return getElbowAngle().plus(getWristAngle());
    }

    /**
     * Is arm at at set point
     */
    public boolean atSetpoint() {
        // Is this okay?  Or does we need some fuzziness?
        // double endpointRotations = getEndpointAngle().getRotations();
        // double goalRotations = pidController.getGoal().position;
        // return Math.abs(endpointRotations - goalRotations) < 0.01;
        return pidController.atGoal();
        // return Math.abs(pidController.getError()) < 0.01;
    }

    /**
     * Is arm at at set point
     * @param mode
     */
    public void setIdleMode(IdleMode mode) {
        elbowMotor.configure(new SparkMaxConfig().idleMode(mode), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }


    /**
     * Uses PID to set angle of end point
     * Calculates elbow angle and actually just sets that
     */
    private void setEndpointAngle(Rotation2d targetAngle) {
        // enpoint = elbow + wrist
        // wrist = - elbow / ELBOW_TO_WRIST_RATIO
        // endpoint = elbow - elbow / ELBOW_TO_WRIST_RATIO
        // endpoint = elbow * (ELBOW_TO_WRIST_RATIO - 1) / ELBOW_TO_WRIST_RATIO
        // elbow = endpoint * ELBOW_TO_WRIST_RATIO / (ELBOW_TO_WRIST_RATIO - 1)
        // double elbowRotations = targetAngle.getRotations() * ELBOW_TO_WRIST_RATIO / (ELBOW_TO_WRIST_RATIO - 1);
        // setElbowAngle(Rotation2d.fromRotations(elbowRotations));

        double targetRotations = targetAngle.getRotations();
        endpointSetpointPub.set(targetAngle.getDegrees());
        if (targetRotations > MAX_ELBOW_ANGLE.getRotations()) {
            Elastic.sendNotification(new Notification(
                    NotificationLevel.WARNING, "Setting the elevator height outside of range",
                    "Somebody is messing up the button setting in robot container by setting the height to higher the range",
                    5000));
        }
        if (targetRotations < MIN_ELBOW_ANGLE.getRotations()) {
            Elastic.sendNotification(new Notification(
                    NotificationLevel.WARNING, "Setting the elevator height outside of range",
                    "Somebody is messing up the button setting in robot container by setting the height to lower than 0 (who is doing this???! WTF??)",
                    5000));
        }
        
        targetRotations = MathUtil.clamp(targetRotations, MIN_ELBOW_ANGLE.getRotations(), MAX_ELBOW_ANGLE.getRotations());
        double elbowVoltage = pidController.calculate(getElbowAngle().getRotations(), targetRotations);

        double feedforward = getFeedforward();
        var voltage = elbowVoltage; //  + feedforward;
        voltage = MathUtil.clamp(voltage, -12, 12);
        elbowMotor.setVoltage(voltage);

        // Set for sim
        outputVoltage = voltage;
    }

    public void joystickControl() {
        double voltage = Math.pow(-MathUtil.applyDeadband(RobotContainer.operatorController.getLeftY(), 0.04), 3) * 12;

        voltage = accelLimiter.calculate(voltage);

        double percentHeight = getElbowAngle().getRotations() / (MAX_ELBOW_ANGLE.getRotations() - MIN_ELBOW_ANGLE.getRotations());

        if (voltage < 0) {
            if (percentHeight <= 0.004) {
                voltage = 0;
                accelLimiter.reset(0);
            } else if (percentHeight <= 0.07) {
                voltage = Math.max(voltage, (-12 * Math.pow((percentHeight * (100.0 / SLOW_DOWN_ZONE)), 3.2)) - SLOWEST_SPEED);
            }
        }
        if (voltage > 0) {
            if (percentHeight >= 0.996) {
                voltage = 0;
                accelLimiter.reset(0);
            } else if (percentHeight >= 0.93) {
                voltage = Math.min(voltage, (12 * Math.pow((percentHeight * (100.0 / SLOW_DOWN_ZONE)), 3.2)) + SLOWEST_SPEED);
            }
        }
        voltage = MathUtil.clamp(voltage, -12, 12);

        // voltage = MathUtil.clamp(voltage, -(12 * Math.pow((percentHeight * (100.0 / SLOW_DOWN_ZONE)), 3.0)
        //         - SLOWEST_SPEED), /* lowest voltage allowed */
        //         (12 * ((1 - percentHeight) * (100.0 / SLOW_DOWN_ZONE))) + SLOWEST_SPEED) /* highest voltage allowed */;
        // This clamps the voltage as it gets closer to the the top or the bottom. The
        // slow down zone is the area at the top or the bottom when things.
        // The slowest speed will allow the wrist to still go up and down no mater
        // what as long it has not hit the limit switch
        // Puting it to the power of 3 makes the slowdown more noticable
        elbowMotor.setVoltage(voltage);

        // Set for sim
        outputVoltage = voltage;
    }

    public void stop() {
        elbowMotor.stopMotor();;

        // Set for sim
        outputVoltage = 0;
    }

    public void periodic() {
        elbowDegreesPub.set(getElbowAngle().getDegrees());
        wristDegreesPub.set(getWristAngle().getDegrees());
        endpointDegreesPub.set(getEndpointAngle().getDegrees());
        voltagePub.set(outputVoltage);

        // Visualization
        // Set positions of the stages for visualization
        // Offset arm by elevator height
        // Offset hand by elbow angle
        double elevatorHeight = RobotContainer.elevator.getHeight() - Elevator.STARTING_HEIGHT;
        double elbowRadians = getElbowAngle().getRadians();
        forearmPosePub.set(new Pose3d(0.121, 0, 0.937 + elevatorHeight, new Rotation3d(0, -elbowRadians, 0)));
        handPosePub.set(new Pose3d(0.121 + Math.cos(elbowRadians)*FOREARM_LENGTH, 0, 0.937 + elevatorHeight + Math.sin(elbowRadians)*FOREARM_LENGTH, new Rotation3d(0, -getEndpointAngle().getRadians(), 0)));
    }

    public Command setEnpointAngleCommand(Rotation2d angle) {
        return Commands.run(() -> setEndpointAngle(angle), this); // .until(() -> this.atSetpoint()).finallyDo(this::stop);
    }

    public Command joystickControlCommand() {
        return Commands.run(() -> joystickControl(), this);
    }

    public Command setIdleModeCommand(IdleMode mode) {
        return Commands.runOnce(() -> setIdleMode(mode)).ignoringDisable(true);
    }


    /**
     * Returns the effective lenth to the COM of the arm based on the pivot angle of the wrist 
     */
    public double getEffectiveLength() {
        // Length of hand relative to horizontal
        // The mass of the hand is this distance out from the pivot point of the hand
        double effectiveHandLength = Math.cos(getWristAngle().getRadians()) * HAND_LENGTH;
        handEffectiveLengthPub.set(effectiveHandLength);
        // We need to averag COM of forearm with effective COM of hand
        // So this is (distanceToForarmCOM * massOfForearm + distanceToHandCOM * massOfHand) / totalMass        
        double armCOG = FOREARM_LENGTH/2 * FOREARM_MASS;
        armCOGPub.set(armCOG);
        double handCOG = (FOREARM_LENGTH + effectiveHandLength) * HAND_MASS;
        handCOGPub.set(handCOG);
        double armEffectiveLength = ((FOREARM_LENGTH/2 * FOREARM_MASS + (FOREARM_LENGTH + effectiveHandLength) * HAND_MASS)) / (FOREARM_MASS + HAND_MASS);
        armEffectiveLengthPub.set(armEffectiveLength);
        return armEffectiveLength;
    }

    /**
     * Returns the kG value for the arm
     * kG is the gravitational constant
     * kG = Nominal Voltage * Mass * Radius / (Stall Torque * Gearing)
     * 
     * Stall torques of motors can be found here (from Reca.lc)
     * https://github.com/tervay/recalc/blob/25c08aebd7e28718183a09a1d349267a87daa18d/src/common/models/data/motors.json
     * Neo is 3.28
     */
    public double getKg() {
        return 12 * (FOREARM_MASS + HAND_MASS) * getEffectiveLength() / (3.28 * ELBOW_TO_WRIST_RATIO * MOTOR_TO_ENCODER);
    }

    /**
     * Returns the feedforward value for the arm
     * Feedforward = kS * signum(velocity) + kG * cos(rotations) + kV * velocity + kA * acceleration
     * Since velocity and acceleration setpoints are 0, we can ignore them
     */
    public double getFeedforward() {
        double feedforward = getKg() * Math.cos(getElbowAngle().getRadians());
        // System.out.println("Feedforward: " + 12 * Math.cos(getElbowAngle().getRadians()) * getKg());
        feedforwardPub.set(feedforward);
        return feedforward;
        
    }


    // ---------------------------------------------------------
    // SIMULATION
    @Override
    public void simulationPeriodic() {
        var voltage = MathUtil.clamp(outputVoltage, -RobotController.getBatteryVoltage(), RobotController.getBatteryVoltage());
        armSim.setInput(voltage);
        armSim.update(0.02);

        // Update position of motors
        double elbowRotations = Units.radiansToRotations(armSim.getAngleRads());

        // Set encoders
        elbowEncoderSim.set(elbowRotations);
        wristEncoderSim.set(-elbowRotations / ELBOW_TO_WRIST_RATIO);

        // Simulate load on battery
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
    }
}
