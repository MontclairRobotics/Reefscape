package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
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

public class Elevator3 extends SubsystemBase {

    // -------------------------------------------------------------
    // CONSTANTS
    public static final String RIO_CANBUS = "";  

    public static final int LEFT_MOTOR_ID = 10;
    public static final int RIGHT_MOTOR_ID = 11;

    public static final int BOTTOM_LIMIT_ID = -1;
    public static final int TOP_LIMIT_ID = -1;

    public static final int MOTION_MAGIC_SLOT = 0;
    public static final double MOTION_MAGIC_KS = 0; //.25; // Add 0.25 V output to overcome static friction
    public static final double MOTION_MAGIC_kV = 0; //.12; // A velocity target of 1 rps results in 0.12 V output
    public static final double MOTION_MAGIC_kA = 0; //.01; // An acceleration of 1 rps/s requires 0.01 V output
    public static final double MOTION_MAGIC_kP = 2.5; // A position error of 2.5 rotations results in 12 V output
    public static final double MOTION_MAGIC_kI = 0; // no output for integrated error
    public static final double MOTION_MAGIC_kD = .1; // A velocity error of 1 rps results in 0.1 V output
    public static final double MOTION_MAGIC_kG = 0; // Gravity term
    public static final double MOTION_MAGIC_CRUISE_VELOCITY = 200; // Target cruise velocity of 80 rps
    public static final double MOTION_MAGIC_ACCELERATION = 200; // Target acceleration of 160 rps/s (0.5 seconds)
    public static final double MOTION_MAGIC_JERK = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    public static final double ELEVATOR_GEAR_RATIO = 16;
    public static final double ELEVATOR_PULLEY_RADIUS = Units.inchesToMeters(0.9175);
    public static final double ENCODER_ROTATIONS_TO_METERS_RATIO = ELEVATOR_GEAR_RATIO / (2 * Math.PI * ELEVATOR_PULLEY_RADIUS);

    public static final double ELEVATOR_MAX_HEIGHT = Units.inchesToMeters(46.246); 
    public static final double STAGE3_TO_2_HEIGHT = Units.inchesToMeters(23.743); 

    public static final double ELEVATOR_MAX_VELOCITY = 1.0; // IN METERS PER SECOND

    public static final double ELEVATOR_MASS = Units.lbsToKilograms(20);

    public static final double ELEVATOR_VISUALIZATION_MIN_HEIGHT = 1.0; // In canvas units
    public static final double ELEVATOR_VISUALIZATION_MAX_HEIGHT = 3.0; // In canvas units

    public static final double ELEVATOR_SLOW_DOWN_ZONE = 0.1;

    public static final double ELEVATOR_VOLTAGE_RATE_LIMIT = 30;

    // -------------------------------------------------------------
    // HARDWARE
    // Motor Controllers/Encoders
    private TalonFX leftTalonFX;
    private TalonFX rightTalonFX;

    // Limit Switches
    private LimitSwitch bottomLimit;
    private LimitSwitch topLimit;

    // Motion controller
    private MotionMagicVoltage motionMagicPostiion;

    // PID and FF
    private ElevatorFeedforward feedforward;
    private ProfiledPIDController profiledPidController;
    private PIDController pidController;

    // Limit on rate of change in joystick control
    private SlewRateLimiter joystickAccelerationLimiter;
    

    // -------------------------------------------------------------
    // LOGGING
    // Logging to NT
    private DoublePublisher heightPub;
    private DoublePublisher rotationsPub;
    private DoublePublisher setPointHeightPub;
    private DoublePublisher setPointRotationsPub;
    private DoublePublisher leftHeightPub;
    private DoublePublisher rightHeightPub;
    private StructPublisher<Pose3d> stage2PosePub;
    private StructPublisher<Pose3d> stage3PosePub;

    private BooleanPublisher topLimitPub;
    private BooleanPublisher bottomLimitPub;

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


    /**
     * Constructor
     */
    public Elevator3() {
        // Get network table
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable elevatorTable = inst.getTable("Elevator");

        // Set up publishers
        heightPub = elevatorTable.getDoubleTopic("Height").publish();
        rotationsPub = elevatorTable.getDoubleTopic("Rotations").publish();
        setPointHeightPub = elevatorTable.getDoubleTopic("SetPoint Height").publish();
        setPointRotationsPub = elevatorTable.getDoubleTopic("SetPoint Rotations").publish();
        if (RobotContainer.debugMode) {
            leftHeightPub = elevatorTable.getDoubleTopic("Left Height").publish();
            rightHeightPub = elevatorTable.getDoubleTopic("Right Height").publish();
            topLimitPub = elevatorTable.getBooleanTopic("Top Limit").publish();
            bottomLimitPub = elevatorTable.getBooleanTopic("Bottom Limit").publish();
            stage2PosePub = elevatorTable.getStructTopic("Stage2Pose", Pose3d.struct).publish();
            stage3PosePub = elevatorTable.getStructTopic("Stage3Pose", Pose3d.struct).publish();

            if (Utils.isSimulation()) {
                simHeightPub = elevatorTable.getDoubleTopic("Simulated/Height").publish();
                simRotationsPub = elevatorTable.getDoubleTopic("Simulated/Rotations").publish();
                simVelocityPub = elevatorTable.getDoubleTopic("Simulated/Velocity").publish();
                simVoltagePub = elevatorTable.getDoubleTopic("Simulated/Voltage").publish();
            }
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
        leftTalonFX.getConfigurator().apply(slot0Configs);
        rightTalonFX.getConfigurator().apply(slot0Configs);

        var motionMagicConfigs = elevatorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = MOTION_MAGIC_JERK;
        leftTalonFX.getConfigurator().apply(motionMagicConfigs);
        rightTalonFX.getConfigurator().apply(motionMagicConfigs);

        // Setup limit switches
        bottomLimit = new LimitSwitch(BOTTOM_LIMIT_ID, false);
        topLimit = new LimitSwitch(TOP_LIMIT_ID, false);

        // Setup limit switches
        bottomLimit = new LimitSwitch(BOTTOM_LIMIT_ID, false);
        topLimit = new LimitSwitch(TOP_LIMIT_ID, false);


        // PID and FF
        profiledPidController = new ProfiledPIDController(250, 0, 5, new TrapezoidProfile.Constraints(12,2));
        pidController = new PIDController(250, 0, 5);
        feedforward = new ElevatorFeedforward(0, 0, 0, 0);

        joystickAccelerationLimiter = new SlewRateLimiter(ELEVATOR_VOLTAGE_RATE_LIMIT);

        // Simulation
        if (Utils.isSimulation()) {
            elevatorSim = new ElevatorSim(
                DCMotor.getKrakenX60(2),
                ELEVATOR_GEAR_RATIO,
                ELEVATOR_MASS,
                ELEVATOR_PULLEY_RADIUS,
                0, 
                ELEVATOR_MAX_HEIGHT,
                false,
                0,
                // new double[] {0.01, 0.01} 
                new double[] {0.0, 0.0} 
            );
        }

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
     * Is elevator at this height
     */
    public boolean atHeight(double height) {
        return Math.abs(getHeight() - height) < 0.005;
    }

    /**
     * 
     * @return Height of elevator in meters averaged between the two encoders
     *  if the difference is two much it will send a warning using elastic
     */
    public double getHeight() {
        double leftHeight = leftTalonFX.getRotorPosition().getValueAsDouble() / ENCODER_ROTATIONS_TO_METERS_RATIO;
        double rightHeight = rightTalonFX.getRotorPosition().getValueAsDouble() / ENCODER_ROTATIONS_TO_METERS_RATIO;

        if (Math.abs(leftHeight - rightHeight) < 0.2) {
            Elastic.sendNotification(new Notification(
                    NotificationLevel.WARNING, "Elevator Height Mismatch",
                    "The two elevator encoders give different values :(", 5000));
        }
        double height = (leftHeight + rightHeight) / 2;
        return MathUtil.clamp(height, 0, ELEVATOR_MAX_HEIGHT);
    }



    public void joystickControl() {
        double height = getHeight();
        double feedforwardVoltage = feedforward.calculate(height);
        
        double joystickVoltage = Math.pow(MathUtil.applyDeadband(RobotContainer.operatorController.getLeftY(), 0.04), 3) * 12;

        double voltage = MathUtil.clamp(joystickVoltage + feedforwardVoltage , -12, 12);

        // Rate limit
        voltage = joystickAccelerationLimiter.calculate(voltage);

        // Start slowing down when near top or bottom
        double percentHeight = height / ELEVATOR_MAX_HEIGHT;
        if (percentHeight > (1-ELEVATOR_SLOW_DOWN_ZONE) && voltage > 0) {
            voltage = MathUtil.clamp(voltage, 0, ( 12 * (1 - percentHeight) * (1 / ELEVATOR_SLOW_DOWN_ZONE)));
        }
        if (percentHeight < ELEVATOR_SLOW_DOWN_ZONE && voltage < 0) {
            voltage = MathUtil.clamp(voltage, -( 12 * (percentHeight) * (1 / ELEVATOR_SLOW_DOWN_ZONE)), 0);
        }

        // Stop at top or bottom
        if (isAtTop() && voltage > 0) {
            stop();
        } else if (isAtBottom() && voltage < 0) {
            stop();
        } else {
            // Otherwise set voltage
            leftTalonFX.setVoltage(voltage);
            rightTalonFX.setVoltage(voltage);
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

        double currentHeight = getHeight();
        double feedforwardVoltage = feedforward.calculate(height);

        double pidVoltage = profiledPidController.calculate(currentHeight, height);
        // double pidVoltage = pidController.calculate(currentHeight, height);

        double voltage = MathUtil.clamp(pidVoltage + feedforwardVoltage, -12, 12);
        System.out.println("PID: " + height + " " + currentHeight + " " + pidVoltage + " " + voltage);

        leftTalonFX.setVoltage(voltage);
        rightTalonFX.setVoltage(voltage);
    }
    /**
     * Sets height of the elevator in meters between 0 and the Max height of the elevator
     * @param height Position you want to set the elevator to in meters
     */
    public void setHeightMagicMotion(double height) {
        setPointHeightPub.set(height);
        setPointRotationsPub.set(height * ENCODER_ROTATIONS_TO_METERS_RATIO);
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
        double rotations = height * ENCODER_ROTATIONS_TO_METERS_RATIO; // Converts meters to rotations
        // System.out.println("rotations: " + rotations);

        // Set limits and position
        motionMagicPostiion
            // .withLimitForwardMotion(isAtBottom())
            // .withLimitReverseMotion(isAtTop())
            .withPosition(rotations);

        leftTalonFX.setControl(motionMagicPostiion);
        rightTalonFX.setControl(motionMagicPostiion);
    }

    // ---------------------------------------------------------
    // COMMANDS
    public Command joystickControlCommand() {
        System.out.println("Joystick");
        return Commands.run(() -> joystickControl(), this);
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> stop());
    }

    public Command setHeightCommand(double height) {
        System.out.println("Setting height to " + height);
        return Commands.run(() -> setHeight(height), this); //.until(() -> atHeight(height)).andThen(() -> stop());
        // return Commands.run(() -> setHeightMagicMotion(height), this);
    }

    @Override
    public void periodic() {
        // System.out.println("periodic");
        heightPub.set(getHeight());
        rotationsPub.set((leftTalonFX.getRotorPosition().getValueAsDouble() + rightTalonFX.getRotorPosition().getValueAsDouble()) / 2);
        if (RobotContainer.debugMode) {
            rightHeightPub.set(rightTalonFX.getPosition().getValueAsDouble());
            leftHeightPub.set(leftTalonFX.getPosition().getValueAsDouble());
            topLimitPub.set(topLimit.get());
            bottomLimitPub.set(bottomLimit.get());
        }
        // Set encoders based on if the elevator is at the top of the bottom
        // if (isAtTop()) {
        //     leftTalonFX.setPosition(ELEVATOR_MAX_HEIGHT * ENCODER_ROTATIONS_TO_METERS_RATIO);
        //     rightTalonFX.setPosition(ELEVATOR_MAX_HEIGHT * ENCODER_ROTATIONS_TO_METERS_RATIO);
        // }
        // if (isAtBottom()) {
        //     leftTalonFX.setPosition(0);
        //     rightTalonFX.setPosition(0);
        // }


        // Set positions of the stages for visualization
        // Stage 2 doesn't move until stage 3 is max'd out
        double height = getHeight();
        stage2PosePub.set(new Pose3d(0.103, 0, 0.14 + Math.max(0, height-STAGE3_TO_2_HEIGHT), new Rotation3d()));
        stage3PosePub.set(new Pose3d(0.103, 0, 0.165 + height, new Rotation3d()));
    }

    // ---------------------------------------------------------
    // SIMULATION
    @Override
    public void simulationPeriodic() {
        // Get motor sims
        var leftTalonFXSim = leftTalonFX.getSimState();
        var rightTalonFXSim = rightTalonFX.getSimState();

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
        // NeutralOut means stopped motor and since our motors are in brake mode, elevator shouldn't move
        if (leftTalonFX.getControlMode().getValue() != ControlModeValue.NeutralOut) {
            elevatorSim.setInput(voltage);
            elevatorSim.update(0.02);

            // Update position of motors
            double simMeters = MathUtil.clamp(elevatorSim.getPositionMeters(), 0, ELEVATOR_MAX_HEIGHT);
            simHeightPub.set(simMeters);
            simRotationsPub.set(simMeters * ENCODER_ROTATIONS_TO_METERS_RATIO);
            leftTalonFXSim.setRawRotorPosition(simMeters * ENCODER_ROTATIONS_TO_METERS_RATIO);
            rightTalonFXSim.setRawRotorPosition(simMeters * ENCODER_ROTATIONS_TO_METERS_RATIO);

            // Update velocity of motors
            simVelocityPub.set(elevatorSim.getVelocityMetersPerSecond() * ENCODER_ROTATIONS_TO_METERS_RATIO);
            leftTalonFXSim.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() * ENCODER_ROTATIONS_TO_METERS_RATIO);
            rightTalonFXSim.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() * ENCODER_ROTATIONS_TO_METERS_RATIO);

            // Simulate limit switches
            topLimit.set(elevatorSim.getPositionMeters() >= ELEVATOR_MAX_HEIGHT);
            bottomLimit.set(elevatorSim.getPositionMeters() <= 0);

            // Update 2D mechanism
            elevatorMechanism.setLength(ELEVATOR_VISUALIZATION_MIN_HEIGHT  + (ELEVATOR_MAX_HEIGHT  / elevatorSim.getPositionMeters()) * (ELEVATOR_VISUALIZATION_MAX_HEIGHT - ELEVATOR_VISUALIZATION_MIN_HEIGHT));
        }

    }
}
