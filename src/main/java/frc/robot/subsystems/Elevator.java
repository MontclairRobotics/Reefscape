package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDCommand;
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

public class Elevator extends SubsystemBase {

    // Constants
    private final double METERS_PER_ROTATION = 36*5/1000.0 * (1.0/12.0);
    private final double ROTATIONS_PER_METER = 1/METERS_PER_ROTATION;
    private final static double STARTING_HEIGHT = 0.9718607; // (meters) - distance between top bar and ground (no extension)
    public final static double MAX_HEIGHT = 1.4211 + STARTING_HEIGHT + 0.041; // (meters) - distance between top bar and ground (fully extended)
    private final double MAX_DISPLACEMENT = MAX_HEIGHT - STARTING_HEIGHT; // (meters) - distance bewteen max height and starting height

    private MotionMagicVoltage mm_req;

    private final int LEFT_MOTOR_ID = 20;
    private final int RIGHT_MOTOR_ID = 21;

    // Motor Controllers/Encoders
    private TalonFX leftTalonFX;
    private TalonFX rightTalonFX;

    private Slot0Configs slot0Configs;
    private ElevatorFeedforward elevatorFeedforward;
    

    // Limit Switches
    // private LimitSwitch bottomLimit;
    // private LimitSwitch topLimit;

    // Logging to NT
    DoublePublisher heightPub;
    DoublePublisher leftHeightPub;
    DoublePublisher rightHeightPub;

    // BooleanPublisher topLimitPub;
    // BooleanPublisher bottomLimitPub;

    public Elevator() {

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable debug = inst.getTable("Debug");

        heightPub = debug.getDoubleTopic("Elevator Height").publish();

        if (RobotContainer.debugMode) {
            leftHeightPub = debug.getDoubleTopic("Elevator Left Height").publish();
            rightHeightPub = debug.getDoubleTopic("Elevator Right Height").publish();
            // topLimitPub = debug.getBooleanTopic("Elevator Top Limit").publish();
            // bottomLimitPub = debug.getBooleanTopic("Elevator Bottom Limit").publish();
        }

        leftTalonFX = new TalonFX(LEFT_MOTOR_ID, "rio"); //TODO: find ports
        rightTalonFX = new TalonFX(RIGHT_MOTOR_ID, "rio");

        slot0Configs = new Slot0Configs()
        .withKP(0).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0).withKG(0);

        elevatorFeedforward = new ElevatorFeedforward(slot0Configs.kS, slot0Configs.kG, slot0Configs.kV);

        //Configures Elevator with Slot 0 Configs ^^
        TalonFXConfiguration elevatorConfigs = new TalonFXConfiguration().withSlot0(slot0Configs);

        // set Motion Magic settings
        var motionMagicConfigs = elevatorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

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
    public double getHeight() {
        double leftDisplacement = (leftTalonFX.getPosition().getValueAsDouble()) * METERS_PER_ROTATION;
        double rightDisplacement = (rightTalonFX.getPosition().getValueAsDouble()) * METERS_PER_ROTATION;

        if (Math.abs(leftDisplacement - rightDisplacement) < 0.06) { //TODO: lower when things get more reliable
            Elastic.sendNotification(new Notification(
                    NotificationLevel.WARNING, "Elevator Height Mismatch",
                    "The two elevator encoders give different values :(", 5000));
        }

        return (leftDisplacement + rightDisplacement) / 2.0;
    }

    // public double getHeight() {
    //     return getDisplacedHeight() + STARTING_HEIGHT;
    // }

    /**
     * Manual Control of the elevator with the joystick
     * Will slow down when nearing the top or bottom
     * 
     */
    public void joystickControl() {
        SlewRateLimiter accelerationLimiter = new SlewRateLimiter(4); //TODO: actually set this
        // final MotionMagicVoltage request = new MotionMagicVoltage(0)
        // .withSlot(0);
        
        double voltage = Math.pow(-MathUtil.applyDeadband(RobotContainer.operatorController.getLeftY(), 0.04), 3) * 12;

        accelerationLimiter.calculate(voltage);

        double percentHeight = this.getHeight() / MAX_DISPLACEMENT;
        System.out.println(voltage);
        System.out.println("Percent Height: " + percentHeight);

        voltage = MathUtil.clamp(voltage, -(12 * (percentHeight) * (100.0 / 7.0)) - 0.1, ( 12 * (1 - percentHeight) * (100.0 / 7.0)) + 0.1);
        //This clamps the voltage as it gets closer to the the top. 7 is because at 7% closer to the top is when it starts clamping. The 0.1 is so it can still always move slowly until limit switch
        
        // speed
        // if (isAtTop() && voltage > 0) {
            // stop();
        // } else if (isAtBottom() && voltage < 0) {
            // stop();
        // } else {
            leftTalonFX.setControl(new VoltageOut(voltage).withEnableFOC(true));
        // }
    }

    public void stop() {
        leftTalonFX.setVoltage(0);
    }

    private final SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(1), // Use dynamic voltage of 1 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("Elevator-State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> {
                double percentHeight = this.getHeight() / MAX_DISPLACEMENT;
                System.out.println(volts);
                System.out.println("Percent Height: " + percentHeight);
                if (percentHeight > 0.93 && volts.in(Volts) > 0) {
                    volts = Volts.of(0);
                    //This clamps the voltage as it gets closer to the the top. 7 is because at 7% closer to the top is when it starts clamping
                }
                if (percentHeight < 0.07 && volts.in(Volts) < 0) {
                    volts = Volts.of(0);  
                }
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
        if (displacement > MAX_DISPLACEMENT) {
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

        displacement = MathUtil.clamp(displacement, 0, MAX_DISPLACEMENT);
        double rotations = displacement * ROTATIONS_PER_METER; // Converts meters to rotations
        
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
        heightPub.set(getHeight());
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