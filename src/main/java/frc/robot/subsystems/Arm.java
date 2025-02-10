package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;

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
    private SlewRateLimiter accelLimiter = new SlewRateLimiter(8); // accelerate fully in ~1.5 seconds (can tune value)

    private final double SLOW_DOWN_ZONE = 7.0; //The percent at the top and bottom of the elevator out of the hight of the elevator extention in which the elevator will slow down to avoid crashing during manual control
    private final double SLOWEST_SPEED = 0.3;

    private DutyCycleEncoder j1Encoder;
    // TODO will there be 2 encoders?
    private DutyCycleEncoder j2Encoder;
    private SparkMax wristMotor;
    public double smallWristAngle;
    public double largeWristAngle;

    private DoublePublisher voltagePub;
    private DoublePublisher largeRotPub;
    private DoublePublisher smallRotPub;

    private DoublePublisher setpointPub;

    public Arm() {
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);
        wristMotor = new SparkMax(-1, MotorType.kBrushless);
        j1Encoder = new DutyCycleEncoder(-1, 1 * ENCODER_TO_LARGE_ANGLE, -1); // 1st number is port, 2nd is range in this case 1 rotation per rotation, 3rd number is offset (whatever number you have to add so it reads zero degrees when horizontal)
        j2Encoder = new DutyCycleEncoder(-1, 1, -1); // 1st # is port, 2nd is ratio to rotations of mechanism (1 here), 3rd is initial offset (TODO to be measured)
        pidController = new ProfiledPIDController(0.0, 0.0, 0.0, constraints);
        
        if (!j1Encoder.isConnected()) {
            Elastic.sendNotification(new Notification(NotificationLevel.ERROR, "Encoder disconnected!", "J1 arm encoder not connected!"));
        }

        if (!j2Encoder.isConnected()) {
            Elastic.sendNotification(new Notification(NotificationLevel.ERROR, "Encoder disconnected!", "J2 arm encoder not connected!"));
        }

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .voltageCompensation(12); //TODO needed?
        
        wristMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        


        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable wristTable = inst.getTable("Wrist");
        voltagePub = wristTable.getDoubleTopic("Wrist Voltage").publish();
        largeRotPub = wristTable.getDoubleTopic("Large Wrist Angle Rotations").publish();
        smallRotPub = wristTable.getDoubleTopic("Small Wrist Angle Rotations").publish();
        setpointPub = wristTable.getDoubleTopic("PID Setpoint - Small Angle Rotations").publish();
    }

    public void stopMotor() {
        wristMotor.stopMotor();
    }

    public void setIdleMode(IdleMode mode) {
        wristMotor.configure(new SparkMaxConfig().idleMode(mode), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /*
     * Returns the angle to the horizontal of the endpoint of the arm in rotations
     */
    public Rotation2d getEndpointAngle() {
        // return Rotation2d.fromRotations(j1Encoder.get() - ((j1Encoder.get() * LARGE_ANGLE_TO_SMALL) + SMALL_ANGLE_WHEN_LARGE_IS_HORIZONTAL.getRotations()));
        // OR
        return Rotation2d.fromRotations(j1Encoder.get() - j2Encoder.get());
    }

    private void setWristAngle(Rotation2d targetAngle) {
        double target = targetAngle.getRotations();
        setpointPub.set(target);
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
        
        target = MathUtil.clamp(target, MIN_ANGLE.getRotations(), MAX_ANGLE.getRotations());
        double wristVoltage = pidController.calculate(getEndpointAngle().getRotations(), target);

        wristVoltage = MathUtil.clamp(wristVoltage, -12, 12);
        // TODO do we need feedforward? If so we have to figure out the equation
        wristMotor.setVoltage(wristVoltage);
    }

    public void joystickControl() {
        double voltage = Math.pow(-MathUtil.applyDeadband(RobotContainer.operatorController.getLeftY(), 0.04), 3) * 12;

        voltage = accelLimiter.calculate(voltage);

        double percentHeight = getEndpointAngle().getRotations() / (MAX_ANGLE.getRotations() - MIN_ANGLE.getRotations());
        // System.out.println(voltage);
        // System.out.println("Percent Height: " + percentHeight);
        // System.out.println("Elevator ff Voltage: " + elevatorFeedforward.calculate(0));

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
        voltagePub.set(voltage);

        // voltage = MathUtil.clamp(voltage, -(12 * Math.pow((percentHeight * (100.0 / SLOW_DOWN_ZONE)), 3.0)
        //         - SLOWEST_SPEED), /* lowest voltage allowed */
        //         (12 * ((1 - percentHeight) * (100.0 / SLOW_DOWN_ZONE))) + SLOWEST_SPEED) /* highest voltage allowed */;
        // This clamps the voltage as it gets closer to the the top or the bottom. The
        // slow down zone is the area at the top or the bottom when things.
        // The slowest speed will allow the wrist to still go up and down no mater
        // what as long it has not hit the limit switch
        // Puting it to the power of 3 makes the slowdown more noticable
        wristMotor.setVoltage(voltage);
    }

    public void stop() {
        wristMotor.stopMotor();;
    }

    public void periodic() {
        largeWristAngle = j1Encoder.get();
        smallWristAngle = j2Encoder.get(); // OR (largeWristAngle * LARGE_ANGLE_TO_SMALL) + SMALL_ANGLE_WHEN_LARGE_IS_HORIZONTAL.getRotations(); //TODO is this 30 right? I dont think so
        largeRotPub.set(largeWristAngle);
        smallRotPub.set(smallWristAngle);
    }

    public Command goToAngleCommand(Rotation2d angle) {
        return Commands.run(() -> setWristAngle(angle), this).until(pidController::atGoal).finallyDo(this::stop);
    }

    public Command joystickControlCommand() {
        return Commands.run(() -> joystickControl(), this);
    }

    public Command setIdleModeCommand(IdleMode mode) {
        return Commands.runOnce(() -> setIdleMode(mode)).ignoringDisable(true);
    }

}
