package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.LimitSwitch;

public class Elevator extends SubsystemBase {
    
    // Constants
    private final double ENCODER_ROTATIONS_TO_METERS_RATIO = 0.5; // TODO: FIND THIS
    public static final double ELEVATOR_MAX_HEIGHT = 2.0; // IN METERS
    
    // Motor Controllers/Encoders
    private TalonFX leftTalonFX;
    private TalonFX rightTalonFX;

    // PID/FeedForward controllers
    private PIDController pidController;
    private ElevatorFeedforward elevatorFeedforward;
    
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

        leftTalonFX = new TalonFX(10, "rio");
        rightTalonFX = new TalonFX(11, "rio");

        pidController = new PIDController(0, 0, 0);
        elevatorFeedforward = new ElevatorFeedforward(0, 0, 0, 0); // TODO: Turn

        bottomLimit = new LimitSwitch(-1, false); // TODO: get port
        topLimit = new LimitSwitch(-1, false);

    }

    public boolean isAtBottom() {
        return bottomLimit.get();
    }

    public double getHeight() { // TODO: maybe average this with the other encoder? see if necessary
        return leftTalonFX.getPosition().getValueAsDouble() * ENCODER_ROTATIONS_TO_METERS_RATIO;
    }

    public boolean isAtTop() {
        return topLimit.get();
    }

    // Manual Control
    public void joystickControl() {
        double voltage = RobotContainer.operatorController.getLeftY() * 12 + elevatorFeedforward.calculate(0); 
        // Multiplying by Max Voltage (12)
                                                                                                       
        //TODO: possibly add rate limiter so we don't crash into the max height at full speed                            
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
        //TODO: add safeties!!! (make sure it doesn't try to set a height bigger than the max height or lower than the min height, etc)
        double rotations = height * ENCODER_ROTATIONS_TO_METERS_RATIO;
        leftTalonFX.setVoltage(pidController.calculate(leftTalonFX.getPosition().getValueAsDouble(), rotations)
                + elevatorFeedforward.calculate(0));
        rightTalonFX.setVoltage(pidController.calculate(rightTalonFX.getPosition().getValueAsDouble(), rotations)
                + elevatorFeedforward.calculate(0));
    }

    // Commands
    public Command joystickControlCommand() {
        return Commands.run(() -> joystickControl(), this);
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> stop());
    }

    public Command setHeightCommand(double height) {
        return Commands.run(() -> setHeight(height));
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
    }
}