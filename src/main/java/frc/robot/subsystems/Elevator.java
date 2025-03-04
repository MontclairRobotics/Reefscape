package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.RobotState;
import frc.robot.util.Elastic;
import frc.robot.subsystems.Ratchet;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;

// import frc.robot.util.LimitSwitch;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import frc.robot.util.LimitSwitch;
import frc.robot.util.Tunable;

public class Elevator extends SubsystemBase {
    // Subsystems
    // Constants
    private static final double METERS_PER_ROTATION = 36 * 5 / 1000.0 * (1.0 / 12.0);
    private static final double ROTATIONS_PER_METER = 1.0 / METERS_PER_ROTATION;
    public static final double STARTING_HEIGHT = 0.98;// 0.9718607; // (meters) - distance between top bar and ground
                                                      // (no
                                                      // extension)
    public static final double MAX_EXTENSION = 1.22; // 1.4189392089843749; // (meters) - distance bewteen max height
                                                     // and
                                                     // starting height
    public static final double MAX_HEIGHT = MAX_EXTENSION + STARTING_HEIGHT; // (meters) - distance between top bar and
                                                                             // ground (fully extended)
    private static final double SLOW_DOWN_ZONE = 7.0; // The percent at the top and bottom of the elevator out of the
                                                      // hight of the elevator extention in which the elevator will slow
                                                      // down to avoid crashing during manual control
    private static final double SLOWEST_SPEED = 0.5; // (IN VOLTAGE) The lowest speed the elevator will go when it
                                                     // thinks it is all the way at the top or bottom of the elevator
                                                     // and is trying to go farther but has not yet hit the limit switch
                                                     // during manual control
    public static final double MAX_VELOCITY_RPS = 100;
    public static final double MAX_ACCEL_RPS = 200;

    public static final double ELEVATOR_PULLEY_RADIUS = Units.inchesToMeters(0.9175);

    // public static final double ELEVATOR_MAX_HEIGHT =
    // Units.inchesToMeters(46.246);

    public static final double STAGE2_MAX_HEIGHT = STARTING_HEIGHT + Units.inchesToMeters(23.743);

    // public static final double STAGE3_TO_2_HEIGHT = Units.inchesToMeters(23.743);

    public static final double ELEVATOR_MASS = Units.lbsToKilograms(40);

    public static final double ELEVATOR_RAISE_TIME = 2;

    public static final double ELEVATOR_VISUALIZATION_MIN_HEIGHT = 1.0; // In canvas units
    public static final double ELEVATOR_VISUALIZATION_MAX_HEIGHT = 3.0; // In canvas units

    private MotionMagicVoltage mm_req;

    private ProfiledPIDController pidController;
    private double lastVelocity = 0;
    private double lastTime = Timer.getFPGATimestamp();

    private final int LEFT_MOTOR_ID = 20;
    private final int RIGHT_MOTOR_ID = 21;

    // Motor Controllers/Encoders
    public TalonFX leftTalonFX;
    public TalonFX rightTalonFX;

    private Slot0Configs slot0Configs;
    private ElevatorFeedforward elevatorFeedforward;

    private double extensionSetpointMeters = 0; // tracks extension setpoint, not from ground

    SlewRateLimiter accelerationLimiter;
    double sysIDVoltage = 0; // TODO delete

    // Limit Switches
    private LimitSwitch bottomLimit;
    private LimitSwitch topLimit;

    // Logging to NT
    DoublePublisher heightPub;
    DoublePublisher percentHeightPub;
    DoublePublisher voltagePub;
    DoublePublisher leftHeightPub;
    DoublePublisher rightHeightPub;

    DoublePublisher setPointPub;

    BooleanPublisher topLimitPub;
    BooleanPublisher bottomLimitPub;

    private StructPublisher<Pose3d> stage2PosePub;
    private StructPublisher<Pose3d> stage3PosePub;

    // -------------------------------------------------------------
    // SIMULATION
    private ElevatorSim elevatorSim;
    private DoublePublisher simHeightPub;
    private DoublePublisher simRotationsPub;
    private DoublePublisher simVelocityPub;
    private DoublePublisher simVoltagePub;

    // -------------------------------------------------------------
    // VISUALIZATION
    private Mechanism2d mechanism;
    private MechanismRoot2d rootMechanism;
    private MechanismLigament2d elevatorMechanism;

    public Elevator() {

        // Tunable kP = new Tunable("kP", 1.5, (val) -> slot0Configs.withKP(val));
        // Tunable kD = new Tunable("kD", 0, (val) -> slot0Configs.withKD(val));

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable elevatorTable = inst.getTable("Elevator");

        heightPub = elevatorTable.getDoubleTopic("Elevator Height").publish();
        percentHeightPub = elevatorTable.getDoubleTopic("Elevator Percent Height").publish();
        voltagePub = elevatorTable.getDoubleTopic("Elevator Voltage").publish();
        setPointPub = elevatorTable.getDoubleTopic("Elevator Setpoint").publish();
        stage2PosePub = elevatorTable.getStructTopic("Stage2Pose", Pose3d.struct).publish();
        stage3PosePub = elevatorTable.getStructTopic("Stage3Pose", Pose3d.struct).publish();

        if (Utils.isSimulation()) {
            simHeightPub = elevatorTable.getDoubleTopic("Simulated/Height").publish();
            simRotationsPub = elevatorTable.getDoubleTopic("Simulated/Rotations").publish();
            simVelocityPub = elevatorTable.getDoubleTopic("Simulated/Velocity").publish();
            simVoltagePub = elevatorTable.getDoubleTopic("Simulated/Voltage").publish();
        }

        if (RobotContainer.debugMode) {
            leftHeightPub = elevatorTable.getDoubleTopic("Elevator Left Height").publish();
            rightHeightPub = elevatorTable.getDoubleTopic("Elevator Right Height").publish();
            // topLimitPub = debug.getBooleanTopic("Elevator Top Limit").publish();
            // bottomLimitPub = debug.getBooleanTopic("Elevator Bottom Limit").publish();
        }

        leftTalonFX = new TalonFX(LEFT_MOTOR_ID, "Drivetrain");
        rightTalonFX = new TalonFX(RIGHT_MOTOR_ID, "Drivetrain");

        // slot0Configs = new Slot0Configs()
        // .withKP(1.4973).withKI(0).withKD(0.098147)
        // .withKS(0.058548).withKV(0.10758).withKA(0.0013553).withKG(0.069812)
        slot0Configs = new Slot0Configs()
                .withKP(1.4973).withKI(0).withKD(0.098147)
                .withKS(0.058548).withKV(0.10758).withKA(0.0013553).withKG(0.22)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        // TODO check this for kv
        elevatorFeedforward = new ElevatorFeedforward(slot0Configs.kS, slot0Configs.kG, slot0Configs.kV);

        pidController = new ProfiledPIDController(8.2697, 0, 0.068398,
                new Constraints(MAX_VELOCITY_RPS, MAX_ACCEL_RPS));

        /*
         * Doesn't actually limit acceleration (change in velocity),
         * limits change in voltage, which is directly proportional to change in
         * velocity
         */
        accelerationLimiter = new SlewRateLimiter(5); // TODO: actually set this

        CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(80)
                .withSupplyCurrentLimit(40);
        // Configures Elevator with Slot 0 Configs ^^
        TalonFXConfiguration leftElevatorConfigs = new TalonFXConfiguration().withSlot0(slot0Configs)
                .withCurrentLimits(currentLimitConfigs);
        TalonFXConfiguration rightElevatorConfigs = new TalonFXConfiguration().withSlot0(slot0Configs)
                .withCurrentLimits(currentLimitConfigs);

        // set Motion Magic settings
        var motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = MAX_VELOCITY_RPS; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = MAX_ACCEL_RPS; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 2000; // Target jerk of 1600 rps/s/s (0.1 seconds)

        leftElevatorConfigs.MotionMagic = motionMagicConfigs;
        rightElevatorConfigs.MotionMagic = motionMagicConfigs;

        leftTalonFX.getConfigurator().apply(leftElevatorConfigs);
        rightTalonFX.getConfigurator().apply(rightElevatorConfigs);
        rightTalonFX.setControl(new Follower(LEFT_MOTOR_ID, true));

        leftTalonFX.setNeutralMode(NeutralModeValue.Brake);
        rightTalonFX.setNeutralMode(NeutralModeValue.Brake);
        leftTalonFX.setPosition(0);
        rightTalonFX.setPosition(0); // Setting encoder to 0

        mm_req = new MotionMagicVoltage(0);

        bottomLimit = new LimitSwitch(21, false); // TODO: get port
        topLimit = new LimitSwitch(22, false); // TODO get invert

        // Simulation
        if (Utils.isSimulation()) {
            elevatorSim = new ElevatorSim(
                    DCMotor.getKrakenX60Foc(2),
                    12, // TODO check, should be 1/12?
                    ELEVATOR_MASS,
                    ELEVATOR_PULLEY_RADIUS,
                    0,
                    MAX_EXTENSION,
                    true,
                    0,
                    // new double[] {0.01, 0.01}
                    new double[] { 0.0, 0.0 });
        }

        // Visualization
        mechanism = new Mechanism2d(4, 4);
        rootMechanism = mechanism.getRoot("ElevatorBottom", 2, 0);
        elevatorMechanism = rootMechanism
                .append(new MechanismLigament2d("Elevator", STARTING_HEIGHT, 90));

        Tunable kG = new Tunable("Elevator kG", slot0Configs.kG, (val) -> {
            elevatorFeedforward = new ElevatorFeedforward(elevatorFeedforward.getKs(), val,
                    elevatorFeedforward.getKv());
        });

        Tunable kV = new Tunable("Elevator kV", slot0Configs.kV, (val) -> {
            elevatorFeedforward = new ElevatorFeedforward(elevatorFeedforward.getKs(), elevatorFeedforward.getKg(),
                    val);
        });

        Tunable kP = new Tunable("Elevator kP", 8.2697, (val) -> {
            pidController = new ProfiledPIDController(val, pidController.getI(), pidController.getD(),
                    new Constraints(MAX_VELOCITY_RPS, MAX_ACCEL_RPS));
        });

        Tunable kI = new Tunable("Elevator kI", 0, (val) -> {
            pidController = new ProfiledPIDController(pidController.getP(), val, pidController.getD(),
                    new Constraints(MAX_VELOCITY_RPS, MAX_ACCEL_RPS));
        });

        Tunable kD = new Tunable("Elevator kD", .068398, (val) -> {
            pidController = new ProfiledPIDController(pidController.getP(), pidController.getI(), val,
                    new Constraints(MAX_VELOCITY_RPS, MAX_ACCEL_RPS));
        });
    }

    /**
     * 
     * @param state the mechanism state to raise to
     * @return the amount of estimated time, in seconds, that it will take to raise
     *         the elevator
     */
    public double getRaiseTime(RobotState state) {
        double percentExtension = Math.abs((state.getHeight() - this.getExtension()) / MAX_EXTENSION);
        System.out.println("Height of pose: " + state.getHeight());
        System.out.println("Extension: " + this.getExtension());
        System.out.println("Percent extension: " + percentExtension);
        return Math.pow(percentExtension, 0.3) + 0.8 * percentExtension + 1;
    }

    /*
     * Sets the elevator target to a height in meters off of the floor
     * NOT relative to the starting position
     */
    public void setHeightProfiledPID(double height) {
        setExtensionProfiledPID(height - STARTING_HEIGHT);
    }

    /*
     * Sets the elevator target to a height in meters off of the floor
     * NOT relative to the starting position
     */
    public void setExtensionProfiledPID(double extension) {

        if (extension > MAX_EXTENSION) {
            Elastic.sendNotification(new Notification(
                    NotificationLevel.WARNING, "Setting the elevator height outside of range",
                    "Somebody is messing up the button setting in robot container by setting the height to higher the range",
                    5000));
        }
        if (extension < 0) {
            Elastic.sendNotification(new Notification(
                    NotificationLevel.WARNING, "Setting the elevator height outside of range",
                    "Somebody is messing up the button setting in robot container by setting the height to lower than 0 (who is doing this???! WTF??)",
                    5000));
        }

        extension = MathUtil.clamp(extension, 0, MAX_EXTENSION);
        double goalRotations = extension * ROTATIONS_PER_METER; // Converts meters to rotations

        setPointPub.set(extension);

        double pidVoltage = pidController.calculate(getExtensionRotations(), goalRotations);

        double accel = (pidController.getSetpoint().velocity - lastVelocity) / (Timer.getFPGATimestamp() - lastTime);

        lastTime = Timer.getFPGATimestamp();
        lastVelocity = leftTalonFX.getVelocity().getValueAsDouble();

        // TODO check accel calculations
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/profiled-pidcontroller.html
        double ffVoltage = elevatorFeedforward.calculate(pidController.getSetpoint().velocity, accel);

        double outputVoltage = MathUtil.clamp(pidVoltage + ffVoltage, -12, 12);

        leftTalonFX.setControl(new VoltageOut(outputVoltage).withEnableFOC(true));

    }

    // public boolean isAtTop() {
    // return topLimit.get();
    // }

    // public boolean isAtBottom() {
    // return bottomLimit.get();
    // }

    /**
     * 
     * @return Height of elevator in meters averaged between the two encoders
     *         if the difference is two much it will send a warning using elastic
     *         uwu
     */
    @AutoLogOutput
    public double getExtension() {
        if (Robot.isReal()) {
            return getExtensionRotations() * METERS_PER_ROTATION;
        }
        return elevatorSim.getPositionMeters();
    }

    public double getHeight() {
        return getExtension() + STARTING_HEIGHT;
    }

    public double getVelocity() {
        return (leftTalonFX.getVelocity().getValueAsDouble() + rightTalonFX.getVelocity().getValueAsDouble()) / 2.0;
    }

    public boolean isVelociatated() {
        return Math.abs(getVelocity()) > 0.1;
    }

    public boolean atSetpoint() {
        return Math.abs(extensionSetpointMeters - getExtension()) < Units.inchesToMeters(1); // TODO find threshold
    }

    public double getExtensionRotations() {
        double leftDisplacement = (leftTalonFX.getPosition().getValueAsDouble());
        double rightDisplacement = (rightTalonFX.getPosition().getValueAsDouble());
        if (Math.abs(leftDisplacement - rightDisplacement) > 0.2) { // TODO: lower
            // when things get more reliable
            Elastic.sendNotification(new Notification(
                    NotificationLevel.WARNING, "Elevator Height Mismatch",
                    "The two elevator encoders give different values :(", 5000));
        }

        return (leftDisplacement + rightDisplacement) / 2.0;
    }

    // public double getTotalHeight() {
    // return getDisplacedHeight() + STARTING_HEIGHT;
    // }

    /**
     * Manual Control of the elevator with the joystick
     * Will slow down when nearing the top or bottom
     * 
     */
    public void joystickControl() {
        // extensionSetpointMeters = getExtension(); // TODO needed? Worried atSetpoint
        // will misbehave, though it shouldn't
        // ever be used unless PID control is running
        double voltage = Math.pow(-MathUtil.applyDeadband(RobotContainer.operatorController.getLeftY(), 0.2), 3) * 12;

        // voltage = accelerationLimiter.calculate(voltage);

        double percentExtension = this.getExtension() / MAX_EXTENSION;
        // System.out.println(voltage);
        // System.out.println("Percent Height: " + percentHeight);
        // System.out.println("Elevator ff Voltage: " +
        // elevatorFeedforward.calculate(0));

        voltage = voltage + elevatorFeedforward.calculate(0);
        if (voltage < 0) {
            if (percentExtension <= 0.01) {
                voltage = 0;
                accelerationLimiter.reset(0);
            } else if (percentExtension <= 0.07) {
                voltage = Math.max(voltage, (-12 * Math.pow((percentExtension * (100.0 /
                        SLOW_DOWN_ZONE)), 3.2)) - SLOWEST_SPEED);
            }
        }
        if (voltage > 0) {
            if (percentExtension >= 0.99) {
                voltage = 0;
                accelerationLimiter.reset(0);
            } else if (percentExtension >= 0.93) {
                voltage = Math.min(voltage, (12 * Math.pow((percentExtension * (100.0 /
                        SLOW_DOWN_ZONE)), 3.2)) + SLOWEST_SPEED);
            }
        }
        // System.out.println(voltage);
        // System.out.println("After: " + voltage);
        voltage = MathUtil.clamp(voltage, -12, 12);
        voltagePub.set(voltage);

        // voltage = MathUtil.clamp(voltage, -(12 * Math.pow((percentHeight * (100.0 /
        // SLOW_DOWN_ZONE)), 3.0)
        // - SLOWEST_SPEED), /* lowest voltage allowed */
        // (12 * ((1 - percentHeight) * (100.0 / SLOW_DOWN_ZONE))) + SLOWEST_SPEED) /*
        // highest voltage allowed */;
        // This clamps the voltage as it gets closer to the the top or the bottom. The
        // slow down zone is the area at the top or the bottom when things.
        // The slowest speed will allow the elevator to still go up and down no mater
        // what as long it has not hit the limit switch
        // Puting it to the power of 3 makes the slowdown more noticable

        // speed
        // if (isAtTop() && voltage > 0) {
        // stop();
        // } else if (isAtBottom() && voltage < 0) {
        // stop();
        // } else {
        leftTalonFX.setControl(new VoltageOut(voltage).withEnableFOC(true));
        // }
    }

    public boolean isSysIDSafe() {
        double percentHeight = this.getExtension() / MAX_EXTENSION;
        if (percentHeight > 0.93 && sysIDVoltage > 0) {
            return false; // This clamps the voltage as it gets closer to the the top. 7 is because at 7%
                          // closer to the top is when it starts clamping
        }
        if (percentHeight < 0.07 && sysIDVoltage < 0) {
            return false;
        }
        return true;
    }

    public void stop() {
        leftTalonFX.setVoltage(0);
    }

    public boolean isAtTop() {
        return this.getExtension() / MAX_EXTENSION > 0.93;
    }

    public void setNeutralMode(NeutralModeValue val) {
        leftTalonFX.setNeutralMode(val);
        rightTalonFX.setNeutralMode(val);
    }

    public boolean isAtBottom() {
        return this.getExtension() / MAX_EXTENSION < 0.07;
    }

    private final SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 1 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("Elevator-State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> {
                        double percentHeight = this.getExtension() / MAX_EXTENSION;
                        System.out.println(volts);
                        System.out.println("Percent Height: " + percentHeight);
                        // if (percentHeight > 0.93 && volts.in(Volts) > 0) {
                        // volts = Volts.of(0);
                        // //This clamps the voltage as it gets closer to the the top. 7 is because at
                        // 7% closer to the top is when it starts clamping
                        // }
                        // if (percentHeight < 0.07 && volts.in(Volts) < 0) {
                        // volts = Volts.of(0);
                        // }
                        leftTalonFX.setVoltage(volts.in(Volts));
                    },
                    null,
                    this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    /**
     * Resets the encoders to a rotation. USE CAREFULLY
     * 
     * @param rotationValue
     */
    public void resetEncoders(double rotationValue) {
        leftTalonFX.setPosition(rotationValue);
        rightTalonFX.setPosition(rotationValue);
    }

    /**
     * Sets height of the elevator in meters between 0 and the max height of the
     * elevator
     * 
     * @param extension meters, the distance from top bar to floor
     */
    public void setHeight(double height) {
        if (RobotContainer.ratchet.ratchetEngaged) {
            return;
        }
        double extension = height - STARTING_HEIGHT;
        setExtension(extension);
    }

    /**
     * Sets extension of the elevator in meters between 0 and the max extension of
     * the
     * elevator
     * 
     * @param extension meters, the distance from top bar to starting location of
     *                  top bar
     */
    public void setExtension(double extension) {
        if (RobotContainer.ratchet.ratchetEngaged) {
            return;
        }
        extensionSetpointMeters = extension;
        // if (extension > MAX_EXTENSION) {
        //     Elastic.sendNotification(new Notification(
        //             NotificationLevel.WARNING, "Setting the elevator height outside of range",
        //             "Somebody is messing up the button setting in robot container by setting the height to higher the range",
        //             5000));
        // }
        // if (extension < 0) {
        //     Elastic.sendNotification(new Notification(
        //             NotificationLevel.WARNING, "Setting the elevator height outside of range",
        //             "Somebody is messing up the button setting in robot container by setting the height to lower than 0 (who is doing this???! WTF??)",
        //             5000));
        // }

        extension = MathUtil.clamp(extension, 0, MAX_EXTENSION);
        double rotations = extension * ROTATIONS_PER_METER; // Converts meters to rotations

        setPointPub.set(rotations * METERS_PER_ROTATION);
        Logger.recordOutput("Elevator/Extension Setpoint", rotations * METERS_PER_ROTATION);

        leftTalonFX.setControl(mm_req.withPosition(rotations).withSlot(0).withEnableFOC(true));
    }


    public Command climbUpCommand() {
        return Commands.run(() -> {
            setExtension(RobotState.Climb.getHeight());
            RobotContainer.arm.setEndpointAngle(RobotState.Climb.getAngle());
        }, this);
    }

    public Command climbDownCommand() {
        return Commands.run(() -> {
            RobotContainer.ratchet.engageServos();
            setExtension(0);
        }, this);
    }

    // Commands
    public Command joystickControlCommand() {
        return Commands.run(() -> joystickControl(), this);
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> stop());
    }

    public Command setExtensionCommand(double extension) {
        return Commands.run(() -> setExtension(extension), this);
    }

    public Command setHeightCommand(double height) {
        return Commands.run(() -> setHeight(height), this);
    }

    public Command setState(RobotState state) {
        return Commands.run(() -> setExtension(state.getHeight()), this);
    }

    @AutoLogOutput
    public double getPercentHeight() {
        return getExtension() / MAX_EXTENSION;
    }

    @Override
    public void periodic() {
        
        if (RobotContainer.debugMode && !DriverStation.isFMSAttached()) {
            rightHeightPub.set(rightTalonFX.getPosition().getValueAsDouble());
            leftHeightPub.set(leftTalonFX.getPosition().getValueAsDouble());
            heightPub.set(getExtension());
            percentHeightPub.set(getExtension() / MAX_EXTENSION);
            // topLimitPub.set(false);
            // bottomLimitPub.set(bottomLimit.get());
        }
        // Set encoders based on if the elevator is at the top of the bottom
        // if (isAtTop()) {
        // leftTalonFX.setPosition(MAX_HEIGHT * ROTATIONS_PER_METER);
        // rightTalonFX.setPosition(MAX_HEIGHT * ROTATIONS_PER_METER);
        // }
        // if (isAtBottom()) {
        // leftTalonFX.setPosition(0);
        // rightTalonFX.setPosition(0);
        // } //TODO: check
        if(RobotBase.isSimulation()) {
            double height = getHeight();
            stage2PosePub.set(new Pose3d(-0.103, 0, 0.14 + Math.max(0, height - STAGE2_MAX_HEIGHT), Rotation3d.kZero));
            // stage2PosePub.set(new Pose3d(0, 0, 0 + Math.max(0, height-STAGE2_MAX_HEIGHT),
            // Rotation3d.kZero));
            stage3PosePub.set(new Pose3d(-0.103, 0, 0.165 + height - STARTING_HEIGHT, Rotation3d.kZero));
        }
    }

    // ---------------------------------------------------------
    // SIMULATION
    @Override
    public void simulationPeriodic() {
        // Get motor sims
        var leftTalonFXSim = leftTalonFX.getSimState();
        var rightTalonFXSim = rightTalonFX.getSimState();

        // Make both motors spin the same direction (inverts don't matter in sim)
        rightTalonFXSim.Orientation = ChassisReference.Clockwise_Positive;

        // Set their supply voltage
        leftTalonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        rightTalonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Read their actual voltage (simulated)
        var leftMotorVoltage = leftTalonFXSim.getMotorVoltage();
        var rightMotorVoltage = rightTalonFXSim.getMotorVoltage();
        var voltage = (leftMotorVoltage + rightMotorVoltage) / 2;

        simVoltagePub.set(voltage);
        // System.out.println("simVoltage: " + leftMotorVoltage);

        // Update Sim
        // NeutralOut means stopped motor and since our motors are in brake mode,
        // elevator shouldn't move
        // if (true) {
        elevatorSim.setInput(voltage);
        elevatorSim.update(0.02);

        // Update position of motors
        double simMeters = MathUtil.clamp(elevatorSim.getPositionMeters(), 0, MAX_EXTENSION);
        simHeightPub.set(elevatorSim.getPositionMeters());
        simRotationsPub.set((simMeters) * ROTATIONS_PER_METER);
        leftTalonFXSim.setRawRotorPosition((simMeters) * ROTATIONS_PER_METER);
        rightTalonFXSim.setRawRotorPosition((simMeters) * ROTATIONS_PER_METER);

        // Update velocity of motors
        simVelocityPub.set(elevatorSim.getVelocityMetersPerSecond() * ROTATIONS_PER_METER);
        leftTalonFXSim
                .setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() * ROTATIONS_PER_METER);
        rightTalonFXSim
                .setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() * ROTATIONS_PER_METER);

        // Simulate limit switches
        topLimit.set(elevatorSim.getPositionMeters() >= MAX_HEIGHT);
        bottomLimit.set(elevatorSim.getPositionMeters() <= 0);

        // Update 2D mechanism
        // elevatorMechanism.setLength(
        // ELEVATOR_VISUALIZATION_MIN_HEIGHT + (MAX_HEIGHT /
        // elevatorSim.getPositionMeters())
        // * (ELEVATOR_VISUALIZATION_MAX_HEIGHT - ELEVATOR_VISUALIZATION_MIN_HEIGHT));

        // elevatorMechanism.setLength(elevatorSim.getPositionMeters());

        SmartDashboard.putData("Elevator Mechanism", mechanism);
    }

    // }

}