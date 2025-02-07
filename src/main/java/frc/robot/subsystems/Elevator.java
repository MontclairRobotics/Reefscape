package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.util.Elastic;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;

// import frc.robot.util.LimitSwitch;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import frc.robot.util.Tunable;

public class Elevator extends SubsystemBase {

    // Constants
    private final double METERS_PER_ROTATION = 36*5/1000.0 * (1.0/12.0);
    private final double ROTATIONS_PER_METER = 1.0/METERS_PER_ROTATION;
    public final double STARTING_HEIGHT = 0.9718607; // (meters) - distance between top bar and ground (no extension)
    public final double MAX_EXTENSION = 1.4189392089843749; // (meters) - distance bewteen max height and starting height
    public final double MAX_HEIGHT = MAX_EXTENSION + STARTING_HEIGHT; // (meters) - distance between top bar and ground (fully extended)
    private final double SLOW_DOWN_ZONE = 7.0; //The percent at the top and bottom of the elevator out of the hight of the elevator extention in which the elevator will slow down to avoid crashing during manual control
    private final double SLOWEST_SPEED = 0.3; //(IN VOLTAGE) The lowest speed the elevator will go when it thinks it is all the way at the top or bottom of the elevator and is trying to go farther but has not yet hit the limit switch during manual control
    public final double MAX_VELOCITY_RPS = 100;
    public final double MAX_ACCEL_RPS = 200;

    private MotionMagicVoltage mm_req;

    private ProfiledPIDController pidController;
    private double lastVelocity = 0;
    private double lastTime = Timer.getFPGATimestamp(); 

    private final int LEFT_MOTOR_ID = 20;
    private final int RIGHT_MOTOR_ID = 21;

    // Motor Controllers/Encoders
    private TalonFX leftTalonFX;
    private TalonFX rightTalonFX;

    private Slot0Configs slot0Configs;
    private ElevatorFeedforward elevatorFeedforward;

    double sysIDVoltage = 0; //TODO delete
    
    

    // Limit Switches
    // private LimitSwitch bottomLimit;
    // private LimitSwitch topLimit;

    // Logging to NT
    DoublePublisher heightPub;
    DoublePublisher leftHeightPub;
    DoublePublisher rightHeightPub;

    DoublePublisher setPointPub;

    // BooleanPublisher topLimitPub;
    // BooleanPublisher bottomLimitPub;

    public Elevator() {

        // Tunable kP = new Tunable("kP", 1.5, (val) -> slot0Configs.withKP(val));
        // Tunable kD = new Tunable("kD", 0, (val) -> slot0Configs.withKD(val));

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable elevator = inst.getTable("Elevator");

        heightPub = elevator.getDoubleTopic("Elevator Height").publish();
        setPointPub = elevator.getDoubleTopic("Elevator Setpoint").publish();

        if (RobotContainer.debugMode) {
            leftHeightPub = elevator.getDoubleTopic("Elevator Left Height").publish();
            rightHeightPub = elevator.getDoubleTopic("Elevator Right Height").publish();
            // topLimitPub = debug.getBooleanTopic("Elevator Top Limit").publish();
            // bottomLimitPub = debug.getBooleanTopic("Elevator Bottom Limit").publish();
        }

        leftTalonFX = new TalonFX(LEFT_MOTOR_ID, "rio");
        rightTalonFX = new TalonFX(RIGHT_MOTOR_ID, "rio");

        slot0Configs = new Slot0Configs()
        .withKP(6).withKI(0).withKD(0.56194*0)
        .withKS(0.058548).withKV(0.10855).withKA(0.0015684).withKG(0.061626)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        //TODO check this for kv
        elevatorFeedforward = new ElevatorFeedforward(slot0Configs.kS, slot0Configs.kG, slot0Configs.kV);

        pidController = new ProfiledPIDController(8.2697, 0, 0.068398, new Constraints(MAX_VELOCITY_RPS, MAX_ACCEL_RPS));
        //Configures Elevator with Slot 0 Configs ^^
        TalonFXConfiguration elevatorConfigs = new TalonFXConfiguration().withSlot0(slot0Configs);

        // set Motion Magic settings
        var motionMagicConfigs = elevatorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = MAX_VELOCITY_RPS; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = MAX_ACCEL_RPS; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 2000; // Target jerk of 1600 rps/s/s (0.1 seconds)

        leftTalonFX.getConfigurator().apply(elevatorConfigs);
        rightTalonFX.setControl(new Follower(LEFT_MOTOR_ID, true));

        leftTalonFX.setNeutralMode(NeutralModeValue.Brake);
        rightTalonFX.setNeutralMode(NeutralModeValue.Brake);
        leftTalonFX.setPosition(0);
        rightTalonFX.setPosition(0); //Setting encoder to 0

        mm_req = new MotionMagicVoltage(0);

       //bottomLimit = new LimitSwitch(21, false); // TODO: get port
       //topLimit = new LimitSwitch(22, false);

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


        // TODO check accel calculations https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/profiled-pidcontroller.html
        double ffVoltage = elevatorFeedforward.calculate(pidController.getSetpoint().velocity, accel);

        double outputVoltage =  MathUtil.clamp(pidVoltage + ffVoltage, -12, 12);

        leftTalonFX.setControl(new VoltageOut(outputVoltage).withEnableFOC(true));

    }
    
    // public boolean isAtTop() {
    //     return topLimit.get();
    // }

    // public boolean isAtBottom() {
    //     return bottomLimit.get();
    // }

    /**
     * 
     * @return Height of elevator in meters averaged between the two encoders
     *  if the difference is two much it will send a warning using elastic
     */
    public double getExtension() {
        return getExtensionRotations() * METERS_PER_ROTATION;
    }

    public double getHeight() {
        return getExtension() + STARTING_HEIGHT;
    }

    public double getExtensionRotations() {
        double leftDisplacement = (leftTalonFX.getPosition().getValueAsDouble());
        double rightDisplacement = (rightTalonFX.getPosition().getValueAsDouble());
        // if (Math.abs(leftDisplacement - rightDisplacement) < 0.06) { //TODO: lower when things get more reliable
        //     Elastic.sendNotification(new Notification(
        //             NotificationLevel.WARNING, "Elevator Height Mismatch",
        //             "The two elevator encoders give different values :(", 5000));
        // }

        return (leftDisplacement + rightDisplacement) / 2.0;
    }

    // public double getTotalHeight() {
    //     return getDisplacedHeight() + STARTING_HEIGHT;
    // }

    /**
     * Manual Control of the elevator with the joystick
     * Will slow down when nearing the top or bottom
     * 
     */
    public void joystickControl() {
        SlewRateLimiter accelerationLimiter = new SlewRateLimiter(3); //TODO: actually set this
        // final MotionMagicVoltage request = new MotionMagicVoltage(0)
        // .withSlot(0);
        
        double voltage = Math.pow(-MathUtil.applyDeadband(RobotContainer.operatorController.getLeftY(), 0.04), 3) * 12;

        accelerationLimiter.calculate(voltage);

        double percentHeight = this.getExtension() / MAX_EXTENSION;
        System.out.println(voltage);
        System.out.println("Percent Height: " + percentHeight);
        System.out.println("Elevator ff Voltage: " + elevatorFeedforward.calculate(0));
        voltage = voltage + elevatorFeedforward.calculate(0);


        // TODO this isn't right. It's very close, but at percentHeight = 0.07 the voltage limit is -12.3 (shouldn't actually be a problem, as the input can't be less than -12)
        voltage = MathUtil.clamp(voltage, -(12 * (percentHeight) * (100.0 / SLOW_DOWN_ZONE)) - SLOWEST_SPEED /*lowest voltage allowed*/,
         ( 12 * (1 - percentHeight) * (100.0 / SLOW_DOWN_ZONE)) + SLOWEST_SPEED) /*highest voltage allowed*/;
        //This clamps the voltage as it gets closer to the the top or the bottom. The slow down zone is the area at the top or the bottom when things.
        //The slowest speed will allow the elevator to still go up and down no mater what as long it has not hit the limit switch 
        
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
            return false;            //This clamps the voltage as it gets closer to the the top. 7 is because at 7% closer to the top is when it starts clamping
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
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 1 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("Elevator-State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> {
                double percentHeight = this.getExtension() / MAX_EXTENSION;
                System.out.println(volts);
                System.out.println("Percent Height: " + percentHeight);
                // if (percentHeight > 0.93 && volts.in(Volts) > 0) {
                //     volts = Volts.of(0);
                //     //This clamps the voltage as it gets closer to the the top. 7 is because at 7% closer to the top is when it starts clamping
                // }
                // if (percentHeight < 0.07 && volts.in(Volts) < 0) {
                //     volts = Volts.of(0);  
                // }
                leftTalonFX.setVoltage(volts.in(Volts));
            },
            null,
            this
        )
    );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }
    
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    /**
     * Sets height of the elevator in meters between 0 and the Max height of the elevator
     * @param height meters, the distance from top bar to floor
     */
    public void setHeight(double height) {
        double displacement = height - STARTING_HEIGHT;
        if (displacement > MAX_EXTENSION) {
            Elastic.sendNotification(new Notification(
                    NotificationLevel.WARNING, "Setting the elevator height outside of range",
                    "Somebody is messing up the button setting in robot container by setting the height to higher the range",
                    5000));
        }
        if (displacement < 0) {
            Elastic.sendNotification(new Notification(
                    NotificationLevel.WARNING, "Setting the elevator height outside of range",
                    "Somebody is messing up the button setting in robot container by setting the height to lower than 0 (who is doing this???! WTF??)",
                    5000));
        }

        displacement = MathUtil.clamp(displacement, 0, MAX_EXTENSION);
        double rotations = displacement * ROTATIONS_PER_METER; // Converts meters to rotations
        System.out.println("Target rotations: " + rotations);

        setPointPub.set(displacement);
        
        leftTalonFX.setControl(mm_req.withPosition(rotations).withSlot(0).withEnableFOC(true));
    }

    // Commands
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
        heightPub.set(getExtension());
        if (RobotContainer.debugMode) {
            rightHeightPub.set(rightTalonFX.getPosition().getValueAsDouble());
            leftHeightPub.set(leftTalonFX.getPosition().getValueAsDouble());
            // topLimitPub.set(false);
            // bottomLimitPub.set(bottomLimit.get());
        }
        // Set encoders based on if the elevator is at the top of the bottom
        // if (isAtTop()) {
        //     leftTalonFX.setPosition(MAX_HEIGHT * ROTATIONS_PER_METER);
        //     rightTalonFX.setPosition(MAX_HEIGHT * ROTATIONS_PER_METER);
        // }
        // if (isAtBottom()) {
        //     leftTalonFX.setPosition(0);
        //     rightTalonFX.setPosition(0);
        // } //TODO: check
    }
}