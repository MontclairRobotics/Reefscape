package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Elastic;

import com.ctre.phoenix6.controls.VoltageOut;
// import frc.robot.util.LimitSwitch;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;

public class Elevator extends SubsystemBase {

    // Constants
    private final double METERS_PER_ROTATION = 36*5/1000;
    private final double ELEVATOR_MAX_HEIGHT = 2.0; // IN METERS

    private final int LEFT_MOTOR_ID = 20;
    private final int RIGHT_MOTOR_ID = 21;

    // Motor Controllers/Encoders
    private TalonFX leftTalonFX;
    private TalonFX rightTalonFX;

    // private final double LEFT_ZERO = leftTalonFX.getPosition().getValueAsDouble();
    // private final double RIGHT_ZERO = rightTalonFX.getPosition().getValueAsDouble();

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

        // set slot 0 gains
        // Slot0Configs slot0Configs = elevatorConfigs.Slot0; // TODO: TUNE!!!!
        // slot0Configs.kS = -1; // Add 0.25 V output to overcome static friction
        // slot0Configs.kV = -1; // A velocity target of 1 rps results in 0.12 V output
        // slot0Configs.kA = -1; // An acceleration of 1 rps/s requires 0.01 V output
        // slot0Configs.kP = -1; // A position error of 2.5 rotations results in 12 V output
        // slot0Configs.kI = -1; // no output for integrated error
        // slot0Configs.kD = -1; // A velocity error of 1 rps results in 0.1 V output

        Slot0Configs slot0Configs = new Slot0Configs()
        .withKP(0).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0).withKG(0);

        //Configures Elevator with Slot 0 Configs ^^
        TalonFXConfiguration leftConf = new TalonFXConfiguration().withSlot0(slot0Configs);
        TalonFXConfiguration rightConf = new TalonFXConfiguration().withSlot0(slot0Configs);

        // set Motion Magic settings
        // var motionMagicConfigs = elevatorConfigs.MotionMagic;
        // motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        // motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        // motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        leftTalonFX.getConfigurator().apply(leftConf);
        leftTalonFX.setInverted(false);
        rightTalonFX.getConfigurator().apply(rightConf);
        rightTalonFX.setInverted(true);

        leftTalonFX.setNeutralMode(NeutralModeValue.Brake);
        rightTalonFX.setNeutralMode(NeutralModeValue.Brake);

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
        double leftHeight = (leftTalonFX.getPosition().getValueAsDouble() /*- LEFT_ZERO*/) * METERS_PER_ROTATION;
        double rightHeight = (rightTalonFX.getPosition().getValueAsDouble()/* - RIGHT_ZERO*/) * METERS_PER_ROTATION;

        if (Math.abs(leftHeight - rightHeight) < 0.02) {
            Elastic.sendNotification(new Notification(
                    NotificationLevel.WARNING, "Elevator Height Mismatch",
                    "The two elevator encoders give different values :(", 5000));
        }

        return (leftHeight + rightHeight) / 2;
    }

    /**
     * Manual Control of the elevator with the joystick
     * Will slow down when nearing the top or bottom
     * 
     */
    public void joystickControl() {
        SlewRateLimiter accelerationLimiter = new SlewRateLimiter(0.5); //TODO: actually set this
        // final MotionMagicVoltage request = new MotionMagicVoltage(0)
        // .withSlot(0);
        
        double voltage = Math.pow(-MathUtil.applyDeadband(RobotContainer.operatorController.getLeftY(), 0.04), 3)
        * 12; //+ request.getFeedForwardMeasure().in(Units.Volts);

        voltage = MathUtil.clamp(voltage, -12, 12);
        // accelerationLimiter.calculate(voltage);
        //TODO: Check if the above line is correct

        double percentHeight = this.getHeight() / ELEVATOR_MAX_HEIGHT;
        System.out.println(voltage);
        System.out.println("Percent Height: " + percentHeight);
        // if (percentHeight > 0.93 && voltage > 0) {
        //     voltage = MathUtil.clamp(voltage, 0, ( 12 * (1 - percentHeight) * (100.0 / 7.0)));
        //     //This clamps the voltage as it gets closer to the the top. 7 is because at 7% closer to the top is when it starts clamping
        // }
        // if (percentHeight < 0.07 && voltage < 0) {
        //     voltage = MathUtil.clamp(voltage, -( 12 * (percentHeight) * (100.0 / 7.0)), 0);
        // }

        // speed
        // if (isAtTop() && voltage > 0) {
            // stop();
        // } else if (isAtBottom() && voltage < 0) {
            // stop();
        // } else {
            // leftTalonFX.setControl(new VoltageOut(voltage).withEnableFOC(true));
            // rightTalonFX.setControl(new VoltageOut(voltage).withEnableFOC(true));
        // }
        leftTalonFX.setVoltage(voltage);
        rightTalonFX.setVoltage(voltage);
    }

    public void stop() {
        leftTalonFX.setVoltage(0);
        rightTalonFX.setVoltage(0);
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
        double rotations = height / METERS_PER_ROTATION; // Converts meters to rotations

        final MotionMagicVoltage request = new MotionMagicVoltage(0)
        .withFeedForward(0); //TUNE???? CONFUSTION :(  
        
        leftTalonFX.setControl(request.withPosition(rotations).withFeedForward(0));
        rightTalonFX.setControl(request.withPosition(rotations).withFeedForward(0));
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
        // heightPub.set(getHeight());
        // if (RobotContainer.debugMode) {
        //     rightHeightPub.set(rightTalonFX.getPosition().getValueAsDouble());
        //     leftHeightPub.set(leftTalonFX.getPosition().getValueAsDouble());
        //     topLimitPub.set(false);
        //     bottomLimitPub.set(bottomLimit.get());
        // }
        // // Set encoders based on if the elevator is at the top of the bottom
        // if (isAtTop()) {
        //     leftTalonFX.setPosition(ELEVATOR_MAX_HEIGHT * ENCODER_ROTATIONS_TO_METERS_RATIO);
        //     rightTalonFX.setPosition(ELEVATOR_MAX_HEIGHT * ENCODER_ROTATIONS_TO_METERS_RATIO);
        // }
        // if (isAtBottom()) {
        //     leftTalonFX.setPosition(0);
        //     rightTalonFX.setPosition(0);
        // } TODO: check
    }
}