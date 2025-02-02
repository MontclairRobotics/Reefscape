package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Elastic;
import frc.robot.util.LimitSwitch;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;

public class Elevator2 extends SubsystemBase {

    // -------------------------------------------------------------
    // CONSTANTS
    public static final String RIO_CANBUS = "rio";  

    public static final int LEFT_MOTOR_ID = 10;
    public static final int RIGHT_MOTOR_ID = 11;

    public static final int BOTTOM_LIMIT_ID = -1;
    public static final int TOP_LIMIT_ID = -1;

    public static final int CANDI_ID = -1;

    public static final double ENCODER_ROTATIONS_TO_METERS_RATIO = 0.5; // TODO: FIND THIS
    public static final double ELEVATOR_MAX_HEIGHT = 2.0; // IN METERS
    public static final double ELEVATOR_MAX_VELOCITY = 1.0; // IN METERS PER SECOND

    public static final int MOTION_MAGIC_SLOT = 0;

    public static final double MOTION_MAGIC_KS = -1; // Add 0.25 V output to overcome static friction
    public static final double MOTION_MAGIC_kV = -1; // A velocity target of 1 rps results in 0.12 V output
    public static final double MOTION_MAGIC_kA = -1; // An acceleration of 1 rps/s requires 0.01 V output
    public static final double MOTION_MAGIC_kP = -1; // A position error of 2.5 rotations results in 12 V output
    public static final double MOTION_MAGIC_kI = -1; // no output for integrated error
    public static final double MOTION_MAGIC_kD = -1; // A velocity error of 1 rps results in 0.1 V output
    public static final double MOTION_MAGIC_kG = -1; // Gravity term
    public static final double MOTION_MAGIC_CRUISE_VELOCITY = 80; // Target cruise velocity of 80 rps
    public static final double MOTION_MAGIC_ACCELERATION = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    public static final double MOTION_MAGIC_JERK = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    public static final double ELEVATOR_GEAR_RATIO = 2;
    public static final double ELEVATOR_MASS = 20;
    public static final double ELEVATOR_DRUM_RADIUS = 1;

    public static final double ELEVATOR_VISUALIZATION_MIN_HEIGHT = 1.0; // In canvas units
    public static final double ELEVATOR_VISUALIZATION_MAX_HEIGHT = 3.0; // In canvas units

    // -------------------------------------------------------------
    // HARDWARE
    // Motor Controllers/Encoders
    private TalonFX leftTalonFX;
    private TalonFX rightTalonFX;

    // Motion controller
    private MotionMagicVoltage motionMagicPostiion;
    private MotionMagicVelocityVoltage motionMagicVelocity;

    // Limit Switches
    private LimitSwitch bottomLimit;
    private LimitSwitch topLimit;

    // -------------------------------------------------------------
    // LOGGING
    // Logging to NT
    private DoublePublisher heightPub;
    private DoublePublisher leftHeightPub;
    private DoublePublisher rightHeightPub;

    private BooleanPublisher topLimitPub;
    private BooleanPublisher bottomLimitPub;

    // -------------------------------------------------------------
    // SIMULATION
    private ElevatorSim elevatorSim;

    // -------------------------------------------------------------
    // VISUALIZATION
    private Mechanism2d mechanism;
    private MechanismRoot2d rootMechanism;
    private MechanismLigament2d elevatorMechanism;


    /**
     * Constructor
     */
    public Elevator2() {
        // Get network table
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable debug = inst.getTable("Debug");

        // Set up publishers
        heightPub = debug.getDoubleTopic("Elevator Height").publish();
        if (RobotContainer.debugMode) {
            leftHeightPub = debug.getDoubleTopic("Elevator Left Height").publish();
            rightHeightPub = debug.getDoubleTopic("Elevator Right Height").publish();
            topLimitPub = debug.getBooleanTopic("Elevator Top Limit").publish();
            bottomLimitPub = debug.getBooleanTopic("Elevator Bottom Limit").publish();
        }

        // Create motors
        leftTalonFX = new TalonFX(LEFT_MOTOR_ID, RIO_CANBUS);
        rightTalonFX = new TalonFX(RIGHT_MOTOR_ID, RIO_CANBUS);
        // TODO: Should right motor just be a follower?

        // Turn on brake mode
        leftTalonFX.setNeutralMode(NeutralModeValue.Brake);
        rightTalonFX.setNeutralMode(NeutralModeValue.Brake);

        // Create Magic motion and set config
        motionMagicPostiion = new MotionMagicVoltage(0).withSlot(MOTION_MAGIC_SLOT).withEnableFOC(true);
        motionMagicVelocity = new MotionMagicVelocityVoltage(0).withSlot(MOTION_MAGIC_SLOT).withEnableFOC(true);
    
        // in init function
        TalonFXConfiguration elevatorConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        // TODO: DO we need separate configs for position and velocity?
        Slot0Configs slot0Configs = elevatorConfigs.Slot0;
        slot0Configs.kS = MOTION_MAGIC_KS;
        slot0Configs.kV = MOTION_MAGIC_kV;
        slot0Configs.kA = MOTION_MAGIC_kA;
        slot0Configs.kP = MOTION_MAGIC_kP;
        slot0Configs.kI = MOTION_MAGIC_kI;
        slot0Configs.kD = MOTION_MAGIC_kD;
        slot0Configs.kG = MOTION_MAGIC_kG;

        var motionMagicConfigs = elevatorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = MOTION_MAGIC_JERK;

        // Setup limit switches
        bottomLimit = new LimitSwitch(BOTTOM_LIMIT_ID, false);
        topLimit = new LimitSwitch(TOP_LIMIT_ID, false);


        // If we have a CANdi, we can tie limit switches directly to the motor controller
        // elevatorConfigs.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANdiS1;
        // elevatorConfigs.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANdiS2;
        // elevatorConfigs.HardwareLimitSwitch.ForwardLimitRemoteSensorID = CANDI_ID;
        // elevatorConfigs.HardwareLimitSwitch.ReverseLimitRemoteSensorID = CANDI_ID;

        // Simulation
        elevatorSim = new ElevatorSim(
            DCMotor.getKrakenX60(2),
            ELEVATOR_GEAR_RATIO,
            ELEVATOR_MASS,
            ELEVATOR_DRUM_RADIUS,
            0, 
            ELEVATOR_MAX_HEIGHT,
            true,
            0,
            new double[] {0.01, 0.01} 
        );

        // Visualization
        mechanism = new Mechanism2d(4, 4);
        rootMechanism = mechanism.getRoot("ElevatorBottom", 0, 2);
        elevatorMechanism = rootMechanism.append(new MechanismLigament2d("Elevator", ELEVATOR_VISUALIZATION_MIN_HEIGHT, 90));
    }

    /**
     * Is elevator at top?
     */
    public boolean isAtBottom() {
        return bottomLimit.get();
    }
    /**
     * Is elevator at bottom?
     */
    public boolean isAtTop() {
        return topLimit.get();
    }

    /**
     * 
     * @return Height of elevator in meters averaged between the two encoders
     *  if the difference is two much it will send a warning using elastic
     */
    public double getHeight() {
        double leftHeight = leftTalonFX.getPosition().getValueAsDouble() * ENCODER_ROTATIONS_TO_METERS_RATIO;
        double rightHeight = rightTalonFX.getPosition().getValueAsDouble() * ENCODER_ROTATIONS_TO_METERS_RATIO;

        if (Math.abs(leftHeight - rightHeight) < 0.2) {
            Elastic.sendNotification(new Notification(
                    NotificationLevel.WARNING, "Elevator Height Mismatch",
                    "The two elevator encoders give different values :(", 5000));
        }
        return (leftHeight + rightHeight) / 2;
    }


    /**
     * Manual Control of the elevator with the joystick
     * Will slow down when nearing the top or bottom
     */
    public void joystickControl() {        
        // Get voltage from joystick with a deadband applied
        double velocity = Math.pow(MathUtil.applyDeadband(RobotContainer.operatorController.getLeftY(), 0.04), 3) * ELEVATOR_MAX_VELOCITY;

        double percentHeight = this.getHeight() / ELEVATOR_MAX_HEIGHT;
        if (percentHeight > 0.93 && velocity > 0) {
            velocity = MathUtil.clamp(velocity, 0, ( ELEVATOR_MAX_VELOCITY * (1 - percentHeight) * (100.0 / 7.0)));
            //This clamps the voltage as it gets closer to the the top. 7 is because at 7% closer to the top is when it starts clamping
        }
        if (percentHeight < 0.07 && velocity < 0) {
            velocity = MathUtil.clamp(velocity, -( ELEVATOR_MAX_VELOCITY * (percentHeight) * (100.0 / 7.0)), 0);
        }

        // Redundant limits
        if (isAtTop() && velocity > 0) {
            stop();
        } else if (isAtBottom() && velocity < 0) {
            stop();
        } else {
            // Set limits and velocity
            motionMagicVelocity
                .withLimitForwardMotion(isAtTop())
                .withLimitReverseMotion(isAtBottom())
                .withVelocity(velocity / ENCODER_ROTATIONS_TO_METERS_RATIO);

            // Apply motion magic to motors
            leftTalonFX.setControl(motionMagicVelocity);
            rightTalonFX.setControl(motionMagicVelocity);
        }
    }

    /**
     * Stops the elevator
     */
    public void stop() {
        // TODO: Can break mode hold it steady?  Or do we need to hold position?
        leftTalonFX.stopMotor();
        rightTalonFX.stopMotor();
    }

    /**
     * Sets height of the elevator in meters between 0 and the Max height of the elevator
     * @param height Position you want to set the elevator to in meters
     */
    public void setHeight(double height) {
        if (height > ELEVATOR_MAX_HEIGHT) {
            Elastic.sendNotification(new Notification(
                    NotificationLevel.WARNING, "Setting the elevator height outside of range",
                    "Somebody is messing up the button setting in robot container by setting the height to higher the range",
                    5000));
        }
        if (height < 0) {
            Elastic.sendNotification(new Notification(
                    NotificationLevel.WARNING, "Setting the elevator height outside of range",
                    "Somebody is messing up the button setting in robot container by setting the height to lower than 0 (who is doing this???! WTF??)",
                    5000));
        }
        height = MathUtil.clamp(height, 0, ELEVATOR_MAX_HEIGHT);
        double rotations = height / ENCODER_ROTATIONS_TO_METERS_RATIO; // Converts meters to rotations

        // Set limits and position
        motionMagicPostiion
            .withLimitForwardMotion(isAtBottom())
            .withLimitReverseMotion(isAtTop())
            .withPosition(rotations);

        leftTalonFX.setControl(motionMagicPostiion);
        rightTalonFX.setControl(motionMagicPostiion);
    }

    // ---------------------------------------------------------
    // COMMANDS
    public Command joystickControlCommand() {
        return Commands.run(() -> joystickControl(), this);
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> stop());
    }

    public Command setHeightCommand(double height) {
        return Commands.run(() -> setHeight(height), this);
    }

    @Override
    public void periodic() {
        heightPub.set(getHeight());
        if (RobotContainer.debugMode) {
            rightHeightPub.set(rightTalonFX.getPosition().getValueAsDouble());
            leftHeightPub.set(leftTalonFX.getPosition().getValueAsDouble());
            topLimitPub.set(topLimit.get());
            bottomLimitPub.set(bottomLimit.get());
        }
        // Set encoders based on if the elevator is at the top of the bottom
        if (isAtTop()) {
            leftTalonFX.setPosition(ELEVATOR_MAX_HEIGHT * ENCODER_ROTATIONS_TO_METERS_RATIO);
            rightTalonFX.setPosition(ELEVATOR_MAX_HEIGHT * ENCODER_ROTATIONS_TO_METERS_RATIO);
        }
        if (isAtBottom()) {
            leftTalonFX.setPosition(0);
            rightTalonFX.setPosition(0);
        }
    }

    // ---------------------------------------------------------
    // SIMULATION
    @Override
    public void simulationPeriodic() {
        var leftTalonFXSim = leftTalonFX.getSimState();
        var rightTalonFXSim = rightTalonFX.getSimState();

        leftTalonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        rightTalonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        var leftMotorVoltage = leftTalonFXSim.getMotorVoltage();
        var rightMotorVoltage = rightTalonFXSim.getMotorVoltage();

        elevatorSim.setInput(leftMotorVoltage);
        elevatorSim.update(0.02);

        leftTalonFXSim.setRawRotorPosition(elevatorSim.getPositionMeters() / ENCODER_ROTATIONS_TO_METERS_RATIO);
        rightTalonFXSim.setRawRotorPosition(elevatorSim.getPositionMeters() / ENCODER_ROTATIONS_TO_METERS_RATIO);

        leftTalonFXSim.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() / ENCODER_ROTATIONS_TO_METERS_RATIO);
        rightTalonFXSim.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() / ENCODER_ROTATIONS_TO_METERS_RATIO);

        elevatorMechanism.setLength(ELEVATOR_VISUALIZATION_MIN_HEIGHT  + (ELEVATOR_MAX_HEIGHT  / elevatorSim.getPositionMeters()) * (ELEVATOR_VISUALIZATION_MAX_HEIGHT - ELEVATOR_VISUALIZATION_MIN_HEIGHT));

        topLimit.set(elevatorSim.getPositionMeters() >= ELEVATOR_MAX_HEIGHT);
        bottomLimit.set(elevatorSim.getPositionMeters() <= 0);
    }
}