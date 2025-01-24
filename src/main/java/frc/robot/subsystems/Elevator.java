package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Elastic;
import frc.robot.util.LimitSwitch;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;

public class Elevator extends SubsystemBase {

    // Constants
    private final double ENCODER_ROTATIONS_TO_METERS_RATIO = 0.5; // TODO: FIND THIS
    public static final double ELEVATOR_MAX_HEIGHT = 2.0; // IN METERS

    // Motor Controllers/Encoders
    private TalonFX leftTalonFX;
    private TalonFX rightTalonFX;

    // PID/FeedForward controllers
    // private PIDController pidController;
    // private ElevatorFeedforward elevatorFeedforward;

    // Limit Switches
    private LimitSwitch bottomLimit;
    private LimitSwitch topLimit;

    // Logging to NT
    DoublePublisher heightPub;
    DoublePublisher leftHeightPub;
    DoublePublisher rightHeightPub;

    BooleanPublisher topLimitPub;
    BooleanPublisher bottomLimitPub;

    public Elevator() {

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable debug = inst.getTable("Debug");

        heightPub = debug.getDoubleTopic("Elevator Height").publish();

        if (RobotContainer.debugMode) {
            leftHeightPub = debug.getDoubleTopic("Elevator Left Height").publish();
            rightHeightPub = debug.getDoubleTopic("Elevator Right Height").publish();
            topLimitPub = debug.getBooleanTopic("Elevator Top Limit").publish();
            bottomLimitPub = debug.getBooleanTopic("Elevator Bottom Limit").publish();
        }

        leftTalonFX = new TalonFX(10, "rio"); //TODO: find ports
        rightTalonFX = new TalonFX(11, "rio");

        // pidController = new PIDController(0, 0, 0);

        // in init function
        var elevatorConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = elevatorConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = elevatorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        leftTalonFX.getConfigurator().apply(elevatorConfigs);
        rightTalonFX.getConfigurator().apply(elevatorConfigs);

        // elevatorFeedforward = new ElevatorFeedforward(0, 0, 0, 0); // TODO: Tune

        bottomLimit = new LimitSwitch(-1, false); // TODO: get port
        topLimit = new LimitSwitch(-1, false);
    }

    public boolean isAtBottom() {
        return bottomLimit.get();
    }

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

    public boolean isAtTop() {
        return topLimit.get();
    }

    // Manual Control
    public void joystickControl() {
        // TODO: slew rate limiter
        final MotionMagicVoltage request = new MotionMagicVoltage(0)
        .withFeedForward(0); //TUNE??????? (confusion)
        
        double voltage = RobotContainer.operatorController.getLeftY() * 11 + request.getFeedForwardMeasure().in(Units.Volts);
        //TODO: Check if the above line is correct
        // Multiplying by Max Voltage (12) (ll not 12 because also feedfoward and lazy)

        double percentHeight = this.getHeight() / ELEVATOR_MAX_HEIGHT;
        if (percentHeight > 0.93 && voltage > 0) {
            voltage = MathUtil.clamp(voltage, 0, ( 12 * (1 - percentHeight) * (100.0 / 7.0)));
            //This clamps the voltage as it gets closer to the the top. 7 is because at 7% closer to the top is when it starts clamping
        }
        if (percentHeight < 0.07 && voltage < 0) {
            voltage = MathUtil.clamp(voltage, -( 12 * (percentHeight) * (100.0 / 7.0)), 0);
        }

        // speed
        if (isAtTop() && voltage > 0) {
            stop();
        } else if (isAtBottom() && voltage < 0) {
            stop();
        } else {
            leftTalonFX.setVoltage(voltage);
            rightTalonFX.setVoltage(voltage);
        }
    }

    public void stop() {
        leftTalonFX.setVoltage(0);
        rightTalonFX.setVoltage(0);
    }

    // To Positions
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

        final MotionMagicVoltage request = new MotionMagicVoltage(0)
        .withFeedForward(0); //TUNE???? (CONFUSTION :(  )

        leftTalonFX.setControl(request.withPosition(rotations).withFeedForward(0));
        rightTalonFX.setControl(request.withPosition(rotations).withFeedForward(0));

        // leftTalonFX.setVoltage(pidController.calculate(leftTalonFX.getPosition().getValueAsDouble(), rotations)
        //         + elevatorFeedforward.calculate(0));
        // rightTalonFX.setVoltage(pidController.calculate(rightTalonFX.getPosition().getValueAsDouble(), rotations)
        //         + elevatorFeedforward.calculate(0));
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
            topLimitPub.set(false);
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
}